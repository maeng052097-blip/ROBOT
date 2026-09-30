"""server.py - 웨이퍼 이송셀 CELL_V9 공정 모니터 웹 서버(표준 라이브러리만).

구성:
  - 시뮬 스레드(50Hz): CellSim.tick, 예외 -> ALARM(SIM_EXCEPTION), 캡처/CSV 처리, 스냅샷 갱신
  - HTTP(ThreadingHTTPServer): 정적 파일 / SSE(/events, 20Hz) / /api/state / /api/meta / /camera.jpg / /captures/*
  - 조작 POST /api/cmd : loopback + JSON + Host/Origin + 기동 토큰 + expect_state(409) 모두 통과해야 실행
  - 원격(--lan) 접속 페이지는 view_only (조작부 숨김, 토큰 없음)
웹 '정지' 버튼은 소프트 정지일 뿐 E-STOP 이 아니다(하드웨어 E-STOP 필수).

실행:
  py -3.13 wafer_cell/server.py                     (SIM, 카메라 없음 -> 개념도)
  py -3.13 wafer_cell/server.py --cam 1             (검사 웹캠 인덱스 1)
  py -3.13 wafer_cell/server.py --lan --speed 5     (같은 네트워크에 보기 전용 공개, 5배속)
  브라우저: http://127.0.0.1:8765/   (localhost 는 ::1 로 해석될 수 있어 IP 로 접속)
"""
import argparse
import csv
import ipaddress
import json
import os
import queue
import re
import secrets
import sys
import threading
import time
import pathlib
from datetime import datetime
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlsplit

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from wafer_cell import cell_config as C          # noqa: E402
from wafer_cell import geometry as G             # noqa: E402
from wafer_cell import sequence as Q             # noqa: E402
from wafer_cell.sim import CellSim, CommandError, FAULTS, SIM_ONLY   # noqa: E402
from wafer_cell.camera_feed import NullFeed      # noqa: E402

MIME = {".html": "text/html; charset=utf-8", ".js": "text/javascript; charset=utf-8",
        ".css": "text/css; charset=utf-8", ".svg": "image/svg+xml", ".jpg": "image/jpeg",
        ".png": "image/png", ".json": "application/json", ".ico": "image/x-icon"}
SAFE_NAME = re.compile(r"^[A-Za-z0-9_.-]+$")
CAPTURE_NAME = re.compile(r"^[A-Za-z0-9_.-]+\.jpg$")
LOCAL_HOSTS = ("127.0.0.1", "localhost", "::1")


def jdump(obj):
    return json.dumps(obj, ensure_ascii=True, allow_nan=False, separators=(",", ":")).encode("ascii")


class App:
    def __init__(self, cam=None, port=C.HTTP_PORT, lan=False, speed=1.0, seed=1, record=True, feed=None):
        self.sim = CellSim(seed=seed)
        self.sim.speed = float(speed)
        self.port, self.lan, self.record = int(port), bool(lan), record
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.boot_id = secrets.token_hex(4)
        self.token = secrets.token_urlsafe(18)
        self.log_q = queue.Queue()
        self.snap = {}
        self.csv_done = set()
        if feed is not None:
            self.feed = feed
        elif cam is None:
            self.feed = NullFeed()
        else:
            from wafer_cell.camera_feed import CameraFeed
            self.feed = CameraFeed(int(cam))
        self.httpd = None
        self.threads = []
        self._publish()

    # ------------------------------------------------------------ 시뮬 루프
    def _sim_loop(self):
        period = 1.0 / C.SIM_HZ
        last = time.perf_counter()
        while not self.stop_event.is_set():
            time.sleep(period)
            now = time.perf_counter()
            dt, last = min(now - last, 0.2), now
            with self.lock:
                try:
                    self.sim.tick(dt)
                except Exception as e:                       # 시뮬이 죽어도 화면이 '살아있는 척' 하지 않게
                    self.sim._raise("SIM_EXCEPTION", type(e).__name__)
                self._handle_captures()
                self._record_csv()
                self._publish()

    def _handle_captures(self):
        sim = self.sim
        while sim.pending_captures:
            req = sim.pending_captures.pop(0)
            st = self.feed.status()
            if self.feed.forced_fail or (self.feed.mode == "live" and st["status"] != "OK"):
                sim._raise("CAMERA", st["status"])
                continue
            name = self.feed.capture(req["slot"], req["wafer"])
            sim.captures.append({"slot": req["slot"], "wafer": req["wafer"], "t": req["t"],
                                 "file": None, "pending": name is not None})
        for saved in self.feed.poll_saved():
            for c in reversed(sim.captures):
                if c["wafer"] == saved["wafer"] and c.get("pending"):
                    c["file"], c["pending"] = saved["file"], False
                    break
            for w in sim.wafers:
                if w["id"] == saved["wafer"]:
                    w["capture"] = saved["file"]

    def _record_csv(self):
        if not self.record:
            return
        for w in self.sim.wafers:
            if w["loc"] == "handed_off" and w["id"] not in self.csv_done:
                self.csv_done.add(w["id"])
                try:
                    C.CAPTURE_DIR.mkdir(parents=True, exist_ok=True)
                    path = C.CAPTURE_DIR / f"wafer_log_{datetime.now():%Y%m%d}.csv"
                    new = not path.exists()
                    with open(path, "a", newline="", encoding="utf-8") as f:
                        wr = csv.writer(f)
                        if new:
                            wr.writerow(["boot_id", "source", "wafer_id", "slot", "t_pick", "t_inspect",
                                         "t_out", "t_handoff", "cycle_s", "capture"])
                        wr.writerow([self.boot_id, self.sim.source, w["id"], w["slot"], w["t_pick"],
                                     w["t_inspect"], w["t_out"], w["t_handoff"], w["cycle_s"], w["capture"]])
                except OSError:
                    self.log_q.put("WARN csv write failed")

    def _publish(self):
        s = self.sim.snapshot()
        s["boot_id"] = self.boot_id
        s["cam"] = self.feed.status()
        s["wall"] = time.time()
        self.snap = s
        for e in self.sim.events_since(getattr(self, "_printed", 0)):
            self.log_q.put(f"[{e['t']:8.1f}] {e['level']:5s} {e['code']}")
            self._printed = e["seq"]

    def events_since(self, seq, limit=120):
        with self.lock:
            return self.sim.events_since(seq)[-limit:]

    def meta(self, view_only):
        return {"geometry": C.ui_geometry(), "steps": Q.step_catalog(),
                "groups": Q.GROUP_LABELS, "clearances": G.clearances(),
                "faults": list(FAULTS), "sim_only": list(SIM_ONLY), "view_only": view_only,
                "boot_id": self.boot_id, "source": self.sim.source,
                "cam_mode": self.feed.mode}

    def command(self, body):
        cmd = body.get("cmd")
        args = body.get("args") or {}
        if not isinstance(args, dict):
            raise CommandError("bad_arg", "args 는 객체")
        with self.lock:
            exp = body.get("expect_state")
            if exp and exp != self.sim.state:
                return 409, {"ok": False, "error": "state_changed", "state": self.sim.state,
                             "msg": f"상태가 바뀜({self.sim.state}) - 다시 확인하세요"}
            try:
                res = self.sim.command(cmd, **args)
                if cmd == "fault" and args.get("name") == "camera_fail":
                    self.feed.forced_fail = bool(args.get("on", True))
                self._publish()
                return 200, {"ok": True, "result": res, "state": self.sim.state}
            except CommandError as e:
                return 400, {"ok": False, "error": e.code, "msg": e.msg, "state": self.sim.state}

    # ------------------------------------------------------------ 서버 수명
    def start(self):
        host = "0.0.0.0" if self.lan else C.HTTP_HOST
        self.httpd = CellHTTPServer((host, self.port), Handler)
        self.httpd.app = self
        self.port = self.httpd.server_address[1]
        t1 = threading.Thread(target=self._sim_loop, daemon=True)
        t2 = threading.Thread(target=self.httpd.serve_forever, kwargs={"poll_interval": 0.2}, daemon=True)
        self.threads = [t1, t2]
        for t in self.threads:
            t.start()
        return self

    def stop(self):
        self.stop_event.set()
        if self.httpd:
            self.httpd.shutdown()
            self.httpd.server_close()
        self.feed.close()


class CellHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = os.name != "nt"     # Windows: 이중 실행이 같은 포트를 잡지 못하게

    def handle_error(self, request, client_address):
        if isinstance(sys.exc_info()[1], (ConnectionError, OSError)):
            return
        self.app.log_q.put("ERROR handler exception")


def is_loopback(host):
    try:
        return ipaddress.ip_address(host.split("%")[0]).is_loopback
    except ValueError:
        return False


class Handler(BaseHTTPRequestHandler):
    server_version = "WaferCell/1"
    disable_nagle_algorithm = True

    def log_message(self, fmt, *args):      # 콘솔 출력 안 함(QuickEdit 멈춤 방지)
        pass

    @property
    def app(self):
        return self.server.app

    # ---- 접근 검사
    def _client_local(self):
        return is_loopback(self.client_address[0])

    def _host_name(self):
        h = (self.headers.get("Host") or "").strip()
        if h.startswith("["):
            return h[1:h.find("]")] if "]" in h else h
        return h.rsplit(":", 1)[0] if h.count(":") == 1 else h

    def _host_ok(self):
        name = self._host_name()
        if name in LOCAL_HOSTS:
            return True
        if self.app.lan:
            try:
                ipaddress.ip_address(name)          # DNS 리바인딩 방지: IP 리터럴만
                return True
            except ValueError:
                return False
        return False

    def _view_only(self):
        return not (self._client_local() and self._host_name() in LOCAL_HOSTS)

    def _send(self, code, body, ctype="application/json", extra=None):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-cache")
        self.send_header("X-Content-Type-Options", "nosniff")
        for k, v in (extra or {}).items():
            self.send_header(k, v)
        self.end_headers()
        if self.command != "HEAD":
            self.wfile.write(body)

    def _json(self, code, obj):
        self._send(code, jdump(obj))

    # ---- GET
    def do_GET(self):
        if not self._host_ok():
            return self._json(403, {"ok": False, "error": "bad_host"})
        path = urlsplit(self.path).path
        try:
            if path in ("/", "/index.html"):
                return self._index()
            if path == "/events":
                return self._events()
            if path == "/api/state":
                with self.app.lock:
                    return self._json(200, self.app.snap)
            if path == "/api/meta":
                return self._json(200, self.app.meta(self._view_only()))
            if path == "/camera.jpg":
                seq, buf = self.app.feed.jpeg()
                if not buf:
                    return self._json(404, {"ok": False, "error": "no_frame"})
                return self._send(200, buf, "image/jpeg", {"X-Frame-Seq": str(seq)})
            if path.startswith("/captures/"):
                name = path[len("/captures/"):]
                p = C.CAPTURE_DIR / name
                if not CAPTURE_NAME.match(name) or not p.is_file():
                    return self._json(404, {"ok": False, "error": "not_found"})
                return self._send(200, p.read_bytes(), "image/jpeg")
            name = path.lstrip("/")
            ext = os.path.splitext(name)[1].lower()
            p = C.WEB_DIR / name
            if SAFE_NAME.match(name) and ext in MIME and p.is_file():
                return self._send(200, p.read_bytes(), MIME[ext])
            return self._json(404, {"ok": False, "error": "not_found"})
        except (ConnectionError, OSError):
            return

    def _index(self):
        view_only = self._view_only()
        boot = {"token": None if view_only else self.app.token, "view_only": view_only,
                "boot_id": self.app.boot_id}
        html = (C.WEB_DIR / "index.html").read_text(encoding="utf-8")
        html = html.replace("/*__BOOT__*/null", json.dumps(boot))
        self._send(200, html.encode("utf-8"), MIME[".html"])

    def _events(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("X-Accel-Buffering", "no")
        self.end_headers()
        app = self.app
        period = 1.0 / C.SSE_HZ
        last_tick, last_seq, last_send = -1, 0, 0.0
        try:
            self.wfile.write(b"retry: 1000\n\n")
            while not app.stop_event.is_set():
                snap = app.snap
                now = time.monotonic()
                if snap.get("tick") != last_tick or now - last_send > 1.0:
                    msg = dict(snap)
                    new = app.events_since(last_seq)
                    if new:
                        last_seq = new[-1]["seq"]
                    msg["new_log"] = new
                    self.wfile.write(b"data: " + jdump(msg) + b"\n\n")
                    last_tick, last_send = snap.get("tick"), now
                time.sleep(period)
        except (ConnectionError, OSError):
            return

    def do_HEAD(self):
        if urlsplit(self.path).path == "/events":
            return self._json(405, {"ok": False})
        return self.do_GET()

    # ---- POST
    def do_POST(self):
        path = urlsplit(self.path).path
        if path != "/api/cmd":
            return self._json(404, {"ok": False, "error": "not_found"})
        if not self._client_local():
            return self._json(403, {"ok": False, "error": "view_only", "msg": "보기 전용(원격)"})
        if self._host_name() not in LOCAL_HOSTS:
            return self._json(403, {"ok": False, "error": "bad_host"})
        origin = self.headers.get("Origin")
        if origin:
            o = urlsplit(origin)
            if o.hostname not in LOCAL_HOSTS or (o.port or 80) != self.app.port:
                return self._json(403, {"ok": False, "error": "bad_origin"})
        if (self.headers.get("Content-Type") or "").split(";")[0].strip() != "application/json":
            return self._json(415, {"ok": False, "error": "json_only"})
        if not secrets.compare_digest(self.headers.get("X-Wafer-Token") or "", self.app.token):
            return self._json(403, {"ok": False, "error": "bad_token"})
        try:
            n = int(self.headers.get("Content-Length") or 0)
            if n <= 0 or n > 10000:
                return self._json(400, {"ok": False, "error": "bad_length"})
            body = json.loads(self.rfile.read(n).decode("utf-8"))
            if not isinstance(body, dict):
                raise ValueError
        except (ValueError, UnicodeDecodeError):
            return self._json(400, {"ok": False, "error": "bad_json"})
        code, res = self.app.command(body)
        self._json(code, res)


def main():
    try:                        # cp949 콘솔에 없는 문자가 섞여도 죽지 않게. 출력은 ASCII 만 쓴다.
        sys.stdout.reconfigure(errors="replace")
        sys.stderr.reconfigure(errors="replace")
    except Exception:
        pass
    ap = argparse.ArgumentParser(description="CELL_V9 wafer cell process monitor (SIM)")
    ap.add_argument("--cam", default="none", help="inspection webcam index, or 'none' (default)")
    ap.add_argument("--port", type=int, default=C.HTTP_PORT)
    ap.add_argument("--lan", action="store_true", help="bind 0.0.0.0 (remote = view only)")
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--source", default="sim", choices=["sim", "live"])
    a = ap.parse_args()
    if a.source == "live":
        print("LIVE source is not implemented yet (phase 2: cell controller + protocol). Use --source sim.")
        sys.exit(2)
    cam = None if str(a.cam).lower() == "none" else int(a.cam)
    try:
        app = App(cam=cam, port=a.port, lan=a.lan, speed=a.speed, seed=a.seed).start()
    except OSError as e:
        print(f"cannot start server on port {a.port}: address in use or blocked ({e.__class__.__name__})")
        sys.exit(1)
    print(f"wafer_cell monitor (SIM): http://127.0.0.1:{app.port}/   cam={'none' if cam is None else cam}"
          f"   lan={'on (remote=view only)' if a.lan else 'off'}")
    print("web STOP is a soft stop, NOT an E-STOP. Ctrl+C to quit.")
    try:
        while True:
            try:
                line = app.log_q.get(timeout=0.3)
                print(line.encode("ascii", "replace").decode("ascii"))
            except queue.Empty:
                pass
    except KeyboardInterrupt:
        pass
    finally:
        app.stop()
        print("bye")


if __name__ == "__main__":
    main()
