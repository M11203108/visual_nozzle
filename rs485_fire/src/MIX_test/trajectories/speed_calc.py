import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm

# ── 量測資料與常數 ──────────────────────────────
g = 9.81  # m/s²
angle_to_distance = {
    -25: 2.1, -20: 2.3, -15: 2.7, -10: 3.0,  -5: 3.3,
      0: 3.7,   5: 4.1,  10: 4.4,  15: 4.8,  20: 5.1,
     25: 5.3,  30: 5.5,  35: 5.54
}

def nozzle_height(angle_deg):
    theta = np.radians(angle_deg)
    offset = np.arcsin(5.5 / 22.5)
    return (166 + 22.5 * np.sin(theta + offset)) / 100  # m

def solve_v0(distance, theta, h0):
    def f(V):
        term = V * np.sin(theta)
        return (V * np.cos(theta) / g) * (term + np.sqrt(term**2 + 2 * g * h0)) - distance

    V = 5.0
    for _ in range(100):
        fV  = f(V)
        dV  = (f(V + 0.01) - fV) / 0.01
        if abs(dV) < 1e-6:
            break
        Vn  = V - fV / dV
        if abs(Vn - V) < 1e-4:
            V = Vn
            break
        V = Vn
    return V

# ── 視覺化設定 ─────────────────────────────────
pitch_list = sorted(angle_to_distance)
colors = cm.viridis(np.linspace(0, 1, len(pitch_list)))

fig, ax = plt.subplots(figsize=(10, 6), facecolor='none')  # 透明圖面
ax.set_facecolor('none')                                   # 透明座標軸

ax.axhline(y=1.66, color='gray', ls='--', label="Nozzle Pivot Height (1.66 m)")

# ── 繪製拋物線 ─────────────────────────────────
for idx, angle_deg in enumerate(pitch_list):
    dist  = angle_to_distance[angle_deg]
    theta = np.radians(angle_deg)
    h0    = nozzle_height(angle_deg)
    V0    = solve_v0(dist, theta, h0)

    # 落地時間
    a, b, c = -0.5 * g, V0 * np.sin(theta), h0
    disc    = b**2 - 4*a*c
    if disc < 0:
        continue
    t_land  = max((-b + np.sqrt(disc)) / (2*a),
                  (-b - np.sqrt(disc)) / (2*a))

    # 座標
    t = np.linspace(0, t_land, 100)
    x = V0 * np.cos(theta) * t
    y = h0 + V0 * np.sin(theta) * t - 0.5 * g * t**2

    ax.plot(x, y, color=colors[idx], lw=2, label=f"{angle_deg}°")

# ── 固定視窗：左 7 m、右 −1 m ──────────────────
ax.set_xlim(7, -1)   # ★ 最左 7、最右 -1
ax.set_ylim(0, 3)

# ── 外觀細節 ──────────────────────────────────
ax.set_title("Water-Jet Trajectories ")
ax.set_xlabel("Horizontal Distance X (m)")
ax.set_ylabel("Height Y (m)")
ax.grid(True)
ax.legend(title="Spray Angle")
ax.set_aspect('equal')
fig.tight_layout()

# 若要輸出透明圖片：
# fig.savefig("water_jet_7_to_-1.png", dpi=300, transparent=True)

plt.show()
