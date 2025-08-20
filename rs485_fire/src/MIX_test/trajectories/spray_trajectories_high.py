#!/usr/bin/env python3
# ------------------------------------------------------------
# spray_planner.py – generate pitch/yaw sweep for a fire box,
#                    draw sparse visualization, save angles.
# ------------------------------------------------------------
import numpy as np
import matplotlib.pyplot as plt
import csv, os
import matplotlib as mpl
mpl.rcParams['savefig.transparent'] = True   # ← 讓互動式存檔也透明


# === 0. 物理常數 & 噴流參數 ===
g, v0 = 9.81, 6.5                 # m/s²,  water-jet speed
pitch_range = (-25, 35)            # deg
yaw_range   = (-81, 81)            # deg
pitch_vals  = np.linspace(*pitch_range, 31)    # 掃描解析度（高）
yaw_vals    = np.linspace(*yaw_range, 241)

# 顯示用稀疏解析度
disp_pitch_deg = np.linspace(*pitch_range, 13)  # 13 條彩色落地線
disp_yaws      = np.linspace(*yaw_range, 25)    # 每條 25 點
colors = plt.cm.viridis(np.linspace(0, 1, len(disp_pitch_deg)))

# === 1. 火源設定 ===
cx, cy, cz = -3.21,-0.25,0.0       # box center (m)
lx, ly, lz = 0.04,0.04,0.01      # box size   (m)

plane_mode = 'center'              # 'center' | 'top' | 'custom'
plane_z    = {'center': cz,
              'top'   : cz + lz/2,
              'custom': 1.0}[plane_mode]

# === 2. 幫助函式 ===
def nozzle_height(pitch_deg):
    theta = np.radians(pitch_deg)
    return (167 + 22.5*np.sin(theta + np.arcsin(6/22.5))) / 100.0  # m

def xy_at_height(pitch_deg, yaw_deg, target_z):
    theta, psi = map(np.radians, (pitch_deg, yaw_deg))
    h0 = nozzle_height(pitch_deg)
    a, b, c = -0.5*g, v0*np.sin(theta), h0 - target_z
    disc = b*b - 4*a*c
    if disc < 0: return None
    t_hit = (-b - np.sqrt(disc)) / (2*a)   # 正根
    if t_hit <= 0: return None
    x = -v0*np.cos(theta)*np.cos(psi)*t_hit
    y =  v0*np.cos(theta)*np.sin(psi)*t_hit
    return x, y

def xy_landing(pitch_deg, yaw_deg):
    theta, psi = map(np.radians, (pitch_deg, yaw_deg))
    h0   = nozzle_height(pitch_deg)
    t_ld = (v0*np.sin(theta) + np.sqrt((v0*np.sin(theta))**2 + 2*g*h0)) / g
    d    = v0*np.cos(theta) * t_ld
    return -d*np.cos(psi), d*np.sin(psi)

# === 3. 建立滅火平面 XY 邊界 ===
margin = 0.5
x_min, x_max = cx - lx/2 - margin, cx + lx/2 + margin
y_min, y_max = cy - ly/2 - margin, cy + ly/2 + margin

# === 4. 掃描角度並收集命中 ===
traj_xy, traj_ang = [], []
zigzag = True
for p in pitch_vals:
    yaws = yaw_vals if zigzag else yaw_vals[::-1]
    for y in yaws:
        hit = xy_at_height(p, y, plane_z)
        if hit and x_min <= hit[0] <= x_max and y_min <= hit[1] <= y_max:
            traj_xy.append(hit)
            traj_ang.append((p, y))
    zigzag = not zigzag

print(f"[INFO] 命中角度組數：{len(traj_ang)}")

# === 5. 儲存角度 CSV ===
csv_path = os.path.join(os.path.dirname(__file__), "trajectory_angles.csv")
with open(csv_path, "w", newline="") as f:
    w = csv.writer(f); w.writerow(["pitch_deg","yaw_deg"]); w.writerows(traj_ang)
print(f"[INFO] Saved → {csv_path}")

# === 6. XY 視圖（稀疏落地點 + 藍線）===
fig_xy, ax_xy = plt.subplots(figsize=(8,6), facecolor='none')
ax_xy.set_facecolor('none')

# 落地點：13 條 × 25 點
for i, p in enumerate(disp_pitch_deg):
    xs, ys = [], []
    for y in disp_yaws:
        x, y = xy_landing(p, y)
        xs.append(x); ys.append(y)
    ax_xy.scatter(xs, ys, s=10, color=colors[i], label=f"{p}° ground")

# 藍色掃描軌跡
if traj_xy:
    ax_xy.plot(*zip(*traj_xy), c='deepskyblue', lw=2, label='Sweep path')

# box & nozzle
rect = plt.Rectangle((cx-lx/2, cy-ly/2), lx, ly,
                     ec='firebrick', fc='none', lw=2, ls='--', label='Fire box')
ax_xy.add_patch(rect)
ax_xy.scatter(0, 0, c='blue', marker='x', s=70, label='Nozzle')

ax_xy.set_title(f"XY plane @ z={plane_z:.2f} m  (sparse view)")
ax_xy.set_xlabel("x (m)"); ax_xy.set_ylabel("y (m)")
ax_xy.set_xlim(-7,1); ax_xy.set_ylim(-6,6)
ax_xy.set_aspect('equal'); ax_xy.grid(True); ax_xy.legend(ncol=2, fontsize=8)
plt.tight_layout()

# === 7. XZ 側視圖（稀疏拋物線 + 藍線投影）===
fig_xz, ax_xz = plt.subplots(figsize=(8,4), facecolor='none')
ax_xz.set_facecolor('none')

for i, p in enumerate(disp_pitch_deg):
    th = np.radians(p); h0 = nozzle_height(p)
    t_land = (v0*np.sin(th) + np.sqrt((v0*np.sin(th))**2 + 2*g*h0)) / g
    t = np.linspace(0, t_land, 140)
    x = -v0*np.cos(th)*t
    z =  h0 + v0*np.sin(th)*t - 0.5*g*t**2
    ax_xz.plot(x, z, color=colors[i], lw=0.7)

if traj_xy:
    ax_xz.scatter([pt[0] for pt in traj_xy], [plane_z]*len(traj_xy),
                  c='deepskyblue', s=35, label='Sweep z')
ax_xz.axhline(plane_z, color='k', ls='--', lw=1, alpha=0.5)
ax_xz.scatter(0, nozzle_height(0), c='blue', marker='x', s=70, label='Nozzle')
ax_xz.plot([x_min,x_max,x_max,x_min,x_min], [plane_z]*5,
           'firebrick', lw=2, ls='--', label='Box proj.')
ax_xz.set_title("Side view (X-Z)")
ax_xz.set_xlabel("x (m)"); ax_xz.set_ylabel("z (m)")
ax_xz.set_xlim(-7,1); ax_xz.set_ylim(0, max(plane_z+1,3))
ax_xz.set_aspect('equal'); ax_xz.grid(True); ax_xz.legend(ncol=2, fontsize=8)
plt.tight_layout(); plt.show()
