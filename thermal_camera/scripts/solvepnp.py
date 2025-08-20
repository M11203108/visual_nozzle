#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SolvePnP reprojection-error statistics for 3×11 (或 3×9) 非對稱交錯圓點板
板上最近圓心距離 = 30 mm ⇒ 幾何座標須乘 30/2 = 15 mm
author: your-name
"""

import os, glob
import numpy as np
import cv2

# ---------- 路徑 ----------
DIR = "/home/robot/newjasmine_ws/src/thermal_camera/scripts/dual_calib_20250628_123046"
CAL = np.load(f"{DIR}/calibration_data.npz")
K2, D2 = CAL["K2"], CAL["D2"]                 # 只用 RGB 端

NPZs = sorted(glob.glob(os.path.join(DIR, "points_*.npz")))
assert NPZs, "找不到 points_*.npz 檔案"

SPACING   = 30.0                              # 最近圓心距離 (mm)
SCALE_MM  = SPACING * 0.5                     # ← 非對稱板：obj_points 基本單位 = spacing/2

print(f"📂 {len(NPZs)} 組點位；物點縮放 × {SCALE_MM} mm")

mean_errs, names, all_err = [], [], []

for fn in NPZs:
    data     = np.load(fn, allow_pickle=True)
    obj3d    = data["obj_points"].astype(np.float32) * SCALE_MM     # (N,3)
    img_pts  = data["realsense_img_points"].reshape(-1,2).astype(np.float32)

    # 防呆：確保點數對齊
    if obj3d.shape[0] != img_pts.shape[0]:
        print(f"{os.path.basename(fn)}: 點數不符，跳過")
        continue

    ok, rvec, tvec = cv2.solvePnP(obj3d, img_pts, K2, D2,
                                  flags=cv2.SOLVEPNP_EPNP)
    if not ok:
        print(f"{os.path.basename(fn)} SolvePnP 失敗"); continue

    proj, _ = cv2.projectPoints(obj3d, rvec, tvec, K2, D2)
    err     = np.linalg.norm(proj.reshape(-1,2) - img_pts, axis=1)

    m, M = err.mean(), err.max()
    print(f"{os.path.basename(fn)}: 平均 {m:.2f}px, 最大 {M:.2f}px")

    mean_errs.append(m)
    names.append(os.path.basename(fn))
    all_err.extend(err)

# ---- 統計 ----
all_err = np.asarray(all_err)
p95     = np.percentile(mean_errs, 95)

print("\n📊 SolvePnP (RGB) 統計：")
print(f"➡️  平均：{all_err.mean():.2f}px")
print(f"➡️  最大：{all_err.max():.2f}px")
print(f"➡️  最小：{all_err.min():.2f}px")
print(f"➡️  標準差：{all_err.std():.2f}px")
print(f"\n📌 離群閾值 (95 % 平均)：{p95:.2f}px")

outs = [(n,m) for n,m in zip(names, mean_errs) if m > p95]
if outs:
    print("❌ 離群組：")
    for n,m in outs:
        print(f"  {n:<12} {m:.2f}px")
else:
    print("✅ 無離群組")
