"""PC 쪽 웨이퍼 판정 - SIM 과 LIVE 가 같은 길을 쓴다.

장비(또는 시뮬)가 촬영 요청(CAP id)을 내면 서버가 카메라로 캡처하고 여기서 판정해 JDG(id, OK/NG/ERR, δr, δθ)로 답한다.
판정 알고리즘은 [미구현]: 기본 OK · 치우침 0. '다음 웨이퍼 NG'(수동 지정)는 SIM/LIVE 모두, NG 비율은 SIM 만.
카메라가 실패하면 ERR -> 장비가 CAMERA 알람.
"""
import random

from wafer_cell import cell_config as C


class Judge:
    def __init__(self, seed=1, ng_rate=C.NG_RATE, random_ng=True):
        self.rng = random.Random(seed + 11)
        self.ng_rate, self.random_ng, self.ng_next = float(ng_rate), bool(random_ng), False
        self.last = None

    def decide(self, req, cam_ok=True, faults=()):
        """req = {id, n, w, ...} -> {id, r, dr, dth, why}."""
        if not cam_ok or "camera_fail" in faults:
            res = {"id": req["id"], "r": "ERR", "dr": 0.0, "dth": 0.0, "why": "camera"}
        else:
            ng = self.ng_next or (self.random_ng and self.rng.random() < self.ng_rate)
            why = "manual" if self.ng_next else ("random" if ng else "default")
            self.ng_next = False
            res = {"id": req["id"], "r": "NG" if ng else "OK", "dr": -5.0 if "corr_out" in faults else 0.0,
                   "dth": 0.0, "why": why}
        self.last = dict(res, w=req.get("w"))
        return res
