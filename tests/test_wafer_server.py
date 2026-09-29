"""tests/test_wafer_server.py — 웨이퍼셀 웹 서버 단위테스트(하드웨어/카메라 불필요, 표준 라이브러리만).

임시 포트('127.0.0.1', 0)에 서버를 띄우고:
  - /api/state, /api/meta, 정적 파일 MIME(.js = text/javascript), index 토큰 주입
  - SSE 첫 이벤트(retry + data) 수신
  - 조작 보호: 토큰 없음 403 / JSON 아님 415 / 다른 Origin 403 / 이상한 Host 403 / expect_state 불일치 409
  - 정상 명령(home) 200 -> 원점 복귀 완료
  - 시뮬 내부 예외 -> 서버가 ALARM(SIM_EXCEPTION) 으로 바꾸고 tick 은 계속 증가(화면이 '살아있는 척' 안 함)
  - /captures 경로 이탈 차단
20초 워치독으로 테스트 루프가 멈추지 않게 한다.

실행: py -3.13 tests/test_wafer_server.py
"""
import http.client
import json
import os
import sys
import threading
import time
import pathlib

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))


def req(port, method, path, body=None, headers=None):
    c = http.client.HTTPConnection("127.0.0.1", port, timeout=3)
    h = {"Host": f"127.0.0.1:{port}"}
    h.update(headers or {})
    data = None if body is None else (body if isinstance(body, bytes) else json.dumps(body).encode())
    c.request(method, path, body=data, headers=h)
    r = c.getresponse()
    out = (r.status, r.getheader("Content-Type"), r.read())
    c.close()
    return out


def main():
    wd = threading.Timer(20.0, lambda: (print("WATCHDOG timeout"), os._exit(1)))
    wd.daemon = True
    wd.start()
    from wafer_cell.server import App

    print("test_wafer_server:")
    app = App(cam=None, port=0, record=False, speed=20.0).start()
    port = app.port
    try:
        st, ct, b = req(port, "GET", "/api/state")
        s = json.loads(b)
        assert st == 200 and s["state"] == "IDLE" and s["source"] == "SIM" and s["boot_id"] == app.boot_id
        st, ct, b = req(port, "GET", "/api/meta")
        m = json.loads(b)
        assert m["view_only"] is False and len(m["steps"]) == 27 and m["clearances"][0]["mm"] == 0.5
        st, ct, b = req(port, "GET", "/app.js")
        assert st == 200 and ct.startswith("text/javascript"), ct
        st, ct, b = req(port, "GET", "/")
        assert st == 200 and app.token.encode() in b and b"/*__BOOT__*/" not in b
        print("  OK state / meta / static MIME / token injected for loopback page")

        c = http.client.HTTPConnection("127.0.0.1", port, timeout=2)
        c.request("GET", "/events", headers={"Host": f"127.0.0.1:{port}"})
        r = c.getresponse()
        assert r.status == 200 and r.getheader("Content-Type") == "text/event-stream"
        first = r.fp.readline()
        assert first.startswith(b"retry:"), first
        line = b""
        while not line.startswith(b"data:"):
            line = r.fp.readline()
        ev = json.loads(line[5:])
        assert "tick" in ev and "new_log" in ev and ev["new_log"], ev.keys()
        c.close()
        print("  OK SSE: retry + first data event with log")

        good = {"Content-Type": "application/json", "X-Wafer-Token": app.token}
        assert req(port, "POST", "/api/cmd", {"cmd": "home"}, {"Content-Type": "application/json"})[0] == 403
        assert req(port, "POST", "/api/cmd", {"cmd": "home"}, {"X-Wafer-Token": app.token, "Content-Type": "text/plain"})[0] == 415
        assert req(port, "POST", "/api/cmd", {"cmd": "home"}, dict(good, Origin="http://evil.example"))[0] == 403
        assert req(port, "POST", "/api/cmd", {"cmd": "home"}, dict(good, Host="evil.example"))[0] == 403
        assert req(port, "GET", "/api/state", headers={"Host": "rebind.example"})[0] == 403
        assert req(port, "POST", "/api/cmd", {"cmd": "home", "expect_state": "RUNNING"}, good)[0] == 409
        assert req(port, "POST", "/api/cmd", {"cmd": "start"}, good)[0] == 400
        print("  OK command guards: token / json / origin / host / expect_state / bad state")

        st, _, b = req(port, "POST", "/api/cmd", {"cmd": "home", "expect_state": "IDLE"},
                       dict(good, Origin=f"http://127.0.0.1:{port}"))
        assert st == 200 and json.loads(b)["state"] == "HOMING"
        t0 = time.time()
        while json.loads(req(port, "GET", "/api/state")[2])["state"] != "READY":
            assert time.time() - t0 < 5, "homing too slow"
            time.sleep(0.05)
        print("  OK home via API -> READY")

        assert req(port, "POST", "/api/cmd", {"cmd": "fault", "args": {"name": "sim_exception"}}, good)[0] == 200
        time.sleep(0.3)
        s1 = json.loads(req(port, "GET", "/api/state")[2])
        time.sleep(0.2)
        s2 = json.loads(req(port, "GET", "/api/state")[2])
        assert s1["state"] == "ALARM" and s1["alarm"]["code"] == "SIM_EXCEPTION", s1["alarm"]
        assert s2["tick"] > s1["tick"]
        print("  OK sim exception -> ALARM(SIM_EXCEPTION), tick keeps advancing")

        assert req(port, "GET", "/captures/..%2Fcell_config.py")[0] == 404
        assert req(port, "GET", "/../cell_config.py")[0] == 404
        assert req(port, "GET", "/camera.jpg")[0] == 404
        print("  OK path traversal blocked, no camera frame in SIM")
    finally:
        app.stop()
        wd.cancel()
    print("OK (all passed)")


if __name__ == "__main__":
    main()
