#!/usr/bin/env python3
"""글자로 물체 찾기 (OWLv2) + 모양 따기 (SAM) 서버. ROS 와 따로, ML 전용 환경에서 띄운다 (GPU 필요: 노트북 → 나중에 Jetson).

  ~/capstone_ml/bin/python tools/vision_server.py            # http://127.0.0.1:5578
  (환경: python3 -m venv --system-site-packages ~/capstone_ml && ~/capstone_ml/bin/pip install -U transformers pillow)

navigation/tools/shelf_detect_server.py (선반, 포트 5577) 와 같은 방식 — 둘을 같이 띄워도 된다.
  POST /find?q=a%20banana|a%20basket&sam=1   본문 = JPEG
    → {"boxes": [[x0, y0, x1, y1, 점수, "말"], ...],      점수 높은 순, 픽셀
       "masks": [ "PNG base64" | null, ... ]}               sam=1 이면 말마다 1 등 상자에 SAM 마스크 (나머지 null)
  GET  /health
왜 따로 띄우나: transformers 가 요구하는 Pillow 가 ROS 쪽 시스템 Pillow 보다 새것이라 같은 파이썬에 못 넣음.
모델은 처음 한 번 ~/.cache/huggingface 에 받는다 (OWLv2 600 MB, SAM 360 MB). 인터넷이 없으면 그 폴더를 복사해 오면 된다.
"""
import argparse
import base64
import io
import json
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import parse_qs, urlparse

import numpy as np
import torch
from PIL import Image
from transformers import Owlv2ForObjectDetection, Owlv2Processor, SamModel, SamProcessor


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=5578)
    ap.add_argument("--owl", default="google/owlv2-base-patch16-ensemble")
    ap.add_argument("--sam", default="facebook/sam-vit-base")
    ap.add_argument("--threshold", type=float, default=0.10)
    ap.add_argument("--no-sam", action="store_true", help="SAM 을 안 올림 (GPU 메모리 절약)")
    ap.add_argument("--cpu", action="store_true")
    a = ap.parse_args()
    dev = "cpu" if a.cpu or not torch.cuda.is_available() else "cuda"
    print(f"OWLv2 불러오는 중: {a.owl} ({dev})", flush=True)
    owl_proc = Owlv2Processor.from_pretrained(a.owl)
    owl = Owlv2ForObjectDetection.from_pretrained(a.owl).to(dev).eval()
    sam = sam_proc = None
    if not a.no_sam:
        print(f"SAM 불러오는 중: {a.sam}", flush=True)
        sam_proc = SamProcessor.from_pretrained(a.sam)
        sam = SamModel.from_pretrained(a.sam).to(dev).eval()

    def detect(img, queries):
        inp = owl_proc(text=[queries], images=img, return_tensors="pt").to(dev)
        with torch.no_grad():
            out = owl(**inp)
        side = max(img.size)                              # OWLv2 는 정사각형으로 채운 그림 기준 좌표를 냄
        r = owl_proc.post_process_grounded_object_detection(out, threshold=a.threshold,
                                                            target_sizes=torch.tensor([[side, side]]))[0]
        w, h = img.size
        boxes = []
        for s, l, b in zip(r["scores"].tolist(), r["labels"].tolist(), r["boxes"].tolist()):
            x0, y0, x1, y1 = max(0.0, b[0]), max(0.0, b[1]), min(w - 1.0, b[2]), min(h - 1.0, b[3])
            if x1 > x0 and y1 > y0:
                boxes.append([round(x0, 1), round(y0, 1), round(x1, 1), round(y1, 1), round(s, 3), queries[l]])
        return sorted(boxes, key=lambda b: -b[4])

    def segment(img, boxes):
        """상자들 → 마스크 (H×W bool) 들. 상자마다 SAM 후보 3 개 중 IoU 점수 1 등."""
        inp = sam_proc(img, input_boxes=[[b[:4] for b in boxes]], return_tensors="pt").to(dev)
        with torch.no_grad():
            out = sam(**inp, multimask_output=True)
        masks = sam_proc.post_process_masks(out.pred_masks.cpu(), inp["original_sizes"].cpu(),
                                            inp["reshaped_input_sizes"].cpu())[0]       # [상자, 3, H, W]
        best = out.iou_scores[0].argmax(dim=-1).cpu()                                  # [상자]
        return [masks[i, best[i]].numpy().astype(bool) for i in range(len(boxes))]

    def png_b64(mask):
        buf = io.BytesIO()
        Image.fromarray(mask.astype(np.uint8) * 255).save(buf, format="PNG")
        return base64.b64encode(buf.getvalue()).decode()

    class H(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def _send(self, code, obj):
            body = json.dumps(obj).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            self._send(200, {"ok": True, "owl": a.owl, "sam": None if sam is None else a.sam, "device": dev})

        def do_POST(self):
            u = urlparse(self.path)
            qs = parse_qs(u.query)
            queries = qs.get("q", ["a basket"])[0].split("|")
            want_sam = qs.get("sam", ["0"])[0] == "1" and sam is not None
            data = self.rfile.read(int(self.headers.get("Content-Length", 0)))
            t = time.time()
            try:
                img = Image.open(io.BytesIO(data)).convert("RGB")
                boxes = detect(img, queries)
                t_owl = time.time() - t
                masks = [None] * len(boxes)
                if want_sam and boxes:
                    firsts = {}                            # 말마다 1 등 상자
                    for i, b in enumerate(boxes):
                        firsts.setdefault(b[5], i)
                    idx = sorted(firsts.values())
                    for i, m in zip(idx, segment(img, [boxes[i] for i in idx])):
                        masks[i] = png_b64(m)
            except Exception as e:                        # 그림이 깨졌거나 GPU 메모리 부족 등
                self._send(500, {"error": f"{type(e).__name__}: {e}"})
                return
            print(f"  {len(boxes)} 개, OWLv2 {t_owl:.2f} s" + (f" + SAM {time.time() - t - t_owl:.2f} s" if want_sam else "")
                  + (f" — 1 등 {boxes[0][5]} {boxes[0][4]:.2f}" if boxes else ""), flush=True)
            self._send(200, {"boxes": boxes, "masks": masks})

    print(f"준비됨: http://127.0.0.1:{a.port}/find  (Ctrl+C 로 끔)", flush=True)
    HTTPServer(("127.0.0.1", a.port), H).serve_forever()


if __name__ == "__main__":
    main()
