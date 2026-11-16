#!/usr/bin/env python3
"""OWLv2 한 장 처리 시간 재기 (전처리 / 모델 / 후처리 나눠서). 노트북·Jetson 에서 같은 코드로 비교.

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
from transformers import Owlv2ForObjectDetection, Owlv2Processor

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
    a = ap.parse_args()
    dev = "cuda" if torch.cuda.is_available() else "cpu"
    dtype = torch.float16 if a.fp16 and dev == "cuda" else torch.float32
    t = time.perf_counter()
    proc = Owlv2Processor.from_pretrained(a.model)
    model = Owlv2ForObjectDetection.from_pretrained(a.model, torch_dtype=dtype).to(dev).eval()
    load_s = time.perf_counter() - t
    name = torch.cuda.get_device_name(0) if dev == "cuda" else "cpu"
    print(f"{name}, {dtype}, 모델 불러오기 {load_s:.1f} s, 질의 {len(QUERIES)} 개")

    def sync():
        if dev == "cuda":
            torch.cuda.synchronize()

    imgs = [Image.open(p).convert("RGB") for p in a.images]
    rec = {"pre": [], "model": [], "post": [], "total": []}
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
        best = {}
        for s, l in zip(r["scores"].tolist(), r["labels"].tolist()):
            best[QUERIES[l]] = max(best.get(QUERIES[l], 0.0), s)
        sync()
        t3 = time.perf_counter()
        if k >= a.warmup:
            for key, v in zip(rec, (t1 - t0, t2 - t1, t3 - t2, t3 - t0)):
                rec[key].append(v)
            found[k % len(imgs)] = best
    q = lambda xs, p: sorted(xs)[min(len(xs) - 1, int(p * len(xs)))]  # noqa: E731
    print("단계       평균    중앙값   95%   [s]")
    for key, xs in rec.items():
        print(f"{key:8s} {statistics.mean(xs):7.3f} {statistics.median(xs):7.3f} {q(xs, 0.95):7.3f}")
    for i, p in enumerate(a.images):
        b = found.get(i, {})
        print(f"{p.split('/')[-1]}: 찾음 {len(b)}/{len(QUERIES)} — " + ", ".join(f"{k.split()[-1]} {v:.2f}" for k, v in b.items()))
    if dev == "cuda":
        print(f"GPU 메모리 최대 {torch.cuda.max_memory_allocated() / 1e9:.2f} GB")
    print(json.dumps({k: round(statistics.mean(v), 4) for k, v in rec.items()}))


if __name__ == "__main__":
    main()
