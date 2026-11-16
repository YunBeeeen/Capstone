#!/usr/bin/env python3
"""OWLv2 (+SAM) 한 장 처리 시간 재기 (전처리 / 모델 / 후처리 / SAM 나눠서). 노트북·Jetson 에서 같은 코드로 비교.
--sam <모델> 을 주면 OWLv2 가 말마다 찾은 1 등 상자들로 SAM 을 한 번에 돌려 마스크까지 (vision_server 와 같은 방식).

  python3 tools/bench_owlv2.py img1.jpg img2.jpg --runs 20                 # fp32
  python3 tools/bench_owlv2.py img*.jpg --runs 20 --fp16                   # fp16 (GPU)
  # Jetson (오프라인, 팀 도커):
  docker run --rm --runtime nvidia -e HF_HUB_OFFLINE=1 -v ~/.cache/huggingface:/root/.cache/huggingface \
    -v ~/jetson-bench/owlv2:/w ghcr.io/ravediamond/lerobot-jetson:latest python3 /w/bench_owlv2.py /w/*.jpg --runs 20
"""
import argparse
import json
import statistics
import time

import torch
from PIL import Image
from transformers import Owlv2ForObjectDetection, Owlv2Processor, SamModel, SamProcessor

QUERIES = ["a banana", "a basket", "a carrot", "a corn cob", "an eggplant", "a red chili pepper",
           "a green bell pepper", "a pumpkin", "a sweet potato"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("images", nargs="+")
    ap.add_argument("--model", default="google/owlv2-base-patch16-ensemble")
    ap.add_argument("--runs", type=int, default=20)
    ap.add_argument("--warmup", type=int, default=3)
    ap.add_argument("--fp16", action="store_true")
    ap.add_argument("--threshold", type=float, default=0.15)
    ap.add_argument("--sam", help="SAM 모델 (예: facebook/sam-vit-base 또는 폴더)")
    ap.add_argument("--sam-fp16", action="store_true")
    a = ap.parse_args()
    dev = "cuda" if torch.cuda.is_available() else "cpu"
    dtype = torch.float16 if a.fp16 and dev == "cuda" else torch.float32
    t = time.perf_counter()
    proc = Owlv2Processor.from_pretrained(a.model)
    model = Owlv2ForObjectDetection.from_pretrained(a.model, torch_dtype=dtype).to(dev).eval()
    sam = sam_proc = None
    sam_dtype = torch.float16 if a.sam_fp16 and dev == "cuda" else torch.float32
    if a.sam:
        sam_proc = SamProcessor.from_pretrained(a.sam)
        sam = SamModel.from_pretrained(a.sam, torch_dtype=sam_dtype).to(dev).eval()
    load_s = time.perf_counter() - t
    name = torch.cuda.get_device_name(0) if dev == "cuda" else "cpu"
    print(f"{name}, {dtype}, 모델 불러오기 {load_s:.1f} s, 질의 {len(QUERIES)} 개")

    def sync():
        if dev == "cuda":
            torch.cuda.synchronize()

    imgs = [Image.open(p).convert("RGB") for p in a.images]
    rec = {"pre": [], "model": [], "post": [], "sam": [], "total": []}
    found = {}
    for k in range(a.warmup + a.runs):
        img = imgs[k % len(imgs)]
        sync()
        t0 = time.perf_counter()
        inp = proc(text=[QUERIES], images=img, return_tensors="pt").to(dev)
        if dtype == torch.float16:
            inp["pixel_values"] = inp["pixel_values"].half()
        sync()
        t1 = time.perf_counter()
        with torch.no_grad():
            out = model(**inp)
        sync()
        t2 = time.perf_counter()
        side = max(img.size)
        r = proc.post_process_grounded_object_detection(out, threshold=a.threshold,
                                                        target_sizes=torch.tensor([[side, side]]))[0]
        best, boxes = {}, {}
        for s, l, b in zip(r["scores"].tolist(), r["labels"].tolist(), r["boxes"].tolist()):
            if s > best.get(QUERIES[l], 0.0):
                best[QUERIES[l]], boxes[QUERIES[l]] = s, b
        sync()
        t3 = time.perf_counter()
        n_mask = 0
        if sam is not None and boxes:
            w, h = img.size
            bx = [[max(0.0, b[0]), max(0.0, b[1]), min(w - 1.0, b[2]), min(h - 1.0, b[3])] for b in boxes.values()]
            si = sam_proc(img, input_boxes=[bx], return_tensors="pt").to(dev)
            si["pixel_values"] = si["pixel_values"].to(sam_dtype)
            with torch.no_grad():
                so = sam(**si, multimask_output=True)
            masks = sam_proc.post_process_masks(so.pred_masks.float().cpu(), si["original_sizes"].cpu(),
                                                si["reshaped_input_sizes"].cpu())[0]
            n_mask = masks.shape[0]
        sync()
        t4 = time.perf_counter()
        if k >= a.warmup:
            for key, v in zip(rec, (t1 - t0, t2 - t1, t3 - t2, t4 - t3, t4 - t0)):
                rec[key].append(v)
            if n_mask:
                rec.setdefault("_masks", []).append(n_mask)
            found[k % len(imgs)] = best
    q = lambda xs, p: sorted(xs)[min(len(xs) - 1, int(p * len(xs)))]  # noqa: E731
    print("단계       평균    중앙값   95%   [s]")
    for key, xs in rec.items():
        if key.startswith("_") or (key == "sam" and sam is None):
            continue
        print(f"{key:8s} {statistics.mean(xs):7.3f} {statistics.median(xs):7.3f} {q(xs, 0.95):7.3f}")
    for i, p in enumerate(a.images):
        b = found.get(i, {})
        print(f"{p.split('/')[-1]}: 찾음 {len(b)}/{len(QUERIES)} — " + ", ".join(f"{k.split()[-1]} {v:.2f}" for k, v in b.items()))
    if dev == "cuda":
        print(f"GPU 메모리 최대 {torch.cuda.max_memory_allocated() / 1e9:.2f} GB")
    if "_masks" in rec:
        print(f"SAM 마스크 장당 평균 {statistics.mean(rec['_masks']):.1f} 개 (한 번에)")
    print(json.dumps({k: round(statistics.mean(v), 4) for k, v in rec.items() if not k.startswith("_")}))


if __name__ == "__main__":
    main()
