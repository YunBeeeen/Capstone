#!/usr/bin/env python3
"""녹화된 ZED (SVO2) 한 장으로 물체 찾기: OWLv2 (+SAM) → 깊이 → 로봇 바닥 기준 3D. find_object.py 의 오프라인판.

  ~/capstone_ml/bin/python tools/vision_server.py &                                 # 서버
  ~/capstone_ml/bin/python tools/svo_find.py front.svo2 --lift-mm 349.5 banana basket --frame 0 --out /tmp/svo.jpg

필요: pyzed (ML 환경에 /usr/local/zed/get_python_api.py 로 설치), ZED SDK, GPU (NEURAL 깊이, 처음 한 번 최적화 몇 분).
카메라 자세: 기울기 = 녹화된 IMU (그 장면 시각), 위치 = 로봇 중심 앞 0.167 m, 왼쪽 렌즈 +0.06 m,
  높이 = 0.675 + 리프트 실제 높이 (--lift-mm) + 0.03. 바닥이 충분히 보이면 바닥으로 기울기·높이를 다시 맞춘다.
리프트 실제 높이: collector streams/lift.jsonl 의 position_mm 는 엔코더 값 — 실제는 collector.lift_height_map 으로 바꿔서
  (예: 엔코더 343.7 → 실제 349.5 mm).
"""
import argparse
import base64
import json
import math
import sys
import urllib.parse
import urllib.request
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from arm_kinematics.vision import camera_pose, estimate_object, level_to_floor  # noqa: E402

ZED_HEIGHT_AT_LIFT0, OPTICAL_ABOVE_SCREW, CAM_X, LEFT_LENS_Y = 0.675, 0.03, 0.167, 0.06


def phrase(word):
    w = word.strip()
    return w if " " in w else ("an " if w[0].lower() in "aeiou" else "a ") + w


def read_frame(path, frame, depth_mode="NEURAL"):
    import pyzed.sl as sl
    init = sl.InitParameters()
    init.set_from_svo_file(str(path))
    init.svo_real_time_mode = False
    init.depth_mode = getattr(sl.DEPTH_MODE, depth_mode)
    init.coordinate_units = sl.UNIT.METER
    init.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
    cam = sl.Camera()
    err = cam.open(init)
    if err != sl.ERROR_CODE.SUCCESS:
        raise SystemExit(f"SVO 열기 실패: {err}")
    n = cam.get_svo_number_of_frames()
    cam.set_svo_position(max(0, min(frame, n - 1)))
    rt = sl.RuntimeParameters()
    for _ in range(3):                                    # 첫 grab 은 깊이가 덜 채워질 수 있음
        if cam.grab(rt) != sl.ERROR_CODE.SUCCESS:
            raise SystemExit("SVO grab 실패")
    img, dep, sens = sl.Mat(), sl.Mat(), sl.SensorsData()
    cam.retrieve_image(img, sl.VIEW.LEFT)
    cam.retrieve_measure(dep, sl.MEASURE.DEPTH)
    cam.get_sensors_data(sens, sl.TIME_REFERENCE.IMAGE)
    c = cam.get_camera_information().camera_configuration.calibration_parameters.left_cam
    qx, qy, qz, qw = sens.get_imu_data().get_pose().get_orientation().get()
    pos = cam.get_svo_position()
    cam.close()
    # 쿼터니언 → 회전 (카메라 몸체 x 앞·y 왼쪽·z 위 → 중력 기준)
    R = np.array([[1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
                  [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
                  [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)]])
    pitch = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    roll = math.atan2(R[2, 1], R[2, 2])
    bgr = img.get_data()[:, :, :3].copy()
    depth = dep.get_data().copy()
    depth[~np.isfinite(depth)] = 0.0
    return bgr, depth, (c.fx, c.fy, c.cx, c.cy), roll, pitch, pos, n


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("svo", type=Path)
    ap.add_argument("words", nargs="+")
    ap.add_argument("--frame", type=int, default=0)
    ap.add_argument("--lift-mm", type=float, required=True, help="리프트 실제 높이 [mm] (엔코더 아님)")
    ap.add_argument("--url", default="http://127.0.0.1:5578")
    ap.add_argument("--no-sam", action="store_true")
    ap.add_argument("--min-score", type=float, default=0.15)
    ap.add_argument("--depth-mode", default="NEURAL", help="NEURAL | PERFORMANCE | QUALITY | ULTRA")
    ap.add_argument("--out", type=Path)
    ap.add_argument("--json", type=Path)
    args = ap.parse_args()
    import cv2

    bgr, depth, K, roll, pitch, pos, n = read_frame(args.svo, args.frame, args.depth_mode)
    h = ZED_HEIGHT_AT_LIFT0 + args.lift_mm / 1000 + OPTICAL_ABOVE_SCREW
    T0 = camera_pose(roll, pitch, [CAM_X, LEFT_LENS_Y, h])
    T, lv = level_to_floor(depth, K, T0)
    print(f"{args.svo.name} 장면 {pos}/{n}, {bgr.shape[1]}×{bgr.shape[0]}, IMU 숙임 {math.degrees(pitch):.1f}° 옆 {math.degrees(roll):+.1f}°, "
          f"카메라 높이 {h:.3f} m" + (f" → 바닥 보정 숙임 {lv[0]:+.1f}° 옆 {lv[1]:+.1f}° 높이 {100 * lv[2]:+.1f} cm"
                                   if lv else " (바닥이 충분히 안 보여 보정 안 함)"))
    queries = [phrase(w) for w in args.words]
    ok, jpg = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, 92])
    u = f"{args.url}/find?q={urllib.parse.quote('|'.join(queries))}&sam={0 if args.no_sam else 1}"
    with urllib.request.urlopen(urllib.request.Request(u, data=jpg.tobytes()), timeout=60) as r:
        ans = json.loads(r.read())
    firsts = {}
    for b, m in zip(ans["boxes"], ans["masks"]):
        if b[5] not in firsts and b[4] >= args.min_score:
            firsts[b[5]] = (b, m)
    result = {}
    for q in queries:
        if q not in firsts:
            print(f"{q:24s} 못 찾음")
            continue
        b, m = firsts[q]
        mask = None
        if m:
            mask = cv2.imdecode(np.frombuffer(base64.b64decode(m), np.uint8), cv2.IMREAD_GRAYSCALE) > 0
        est = estimate_object(depth, K, T, b, mask=mask, step=2)
        if est is None:
            print(f"{q:24s} {b[4]:.2f}  3D 실패 (점 부족)")
            continue
        print(f"{q:24s} {b[4]:.2f}  {est}" + (f", 받침면 {100 * est.support_z:.1f} cm" if est.support_z else ""))
        result[q] = {"score": b[4], "box": b[:4], "center": est.center.tolist(), "size": list(est.size), "yaw": est.yaw,
                     "support_z": est.support_z}
        x0, y0, x1, y1 = map(int, b[:4])
        if mask is not None:
            bgr[mask] = (0.5 * bgr[mask] + 0.5 * np.array([0, 200, 255])).astype(np.uint8)
        cv2.rectangle(bgr, (x0, y0), (x1, y1), (0, 0, 255), 3)
        c = est.center
        cv2.putText(bgr, f"{q.split()[-1]} ({100 * c[0]:.0f},{100 * c[1]:.0f},{100 * c[2]:.0f})cm", (x0, max(24, y0 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
    if args.out:
        cv2.imwrite(str(args.out), bgr)
        print("그림:", args.out)
    if args.json:
        args.json.write_text(json.dumps({"camera_T": T.tolist(), "objects": result}, indent=1, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    sys.exit(main())
