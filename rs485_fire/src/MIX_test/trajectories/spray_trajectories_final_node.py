#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
spray_planner_node.py
---------------------
‣ 收集 50 筆 /fire_info
‣ 計算掃描軌跡、輸出 CSV
‣ 持續發佈 /plan_xy_image、/plan_xz_image、/plan_status
‣ 圖像風格與 scripts/spray_planner.py 一致（稀疏彩色落地線）
"""

import os, re, csv, threading, statistics
import numpy as np
import matplotlib as mpl
mpl.use("Agg")                               # GUI 後端，能彈出視窗
mpl.rcParams['savefig.transparent'] = True

import matplotlib.pyplot as plt
from matplotlib.backends.backend_agg import FigureCanvasAgg

import rclpy
from rclpy.node import Node
from rclpy.qos  import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# -------- 參數 / 常數 ----------------------------------------------
SAMPLES   = 20
DATA_RE   = re.compile(r'[-+]?\d*\.\d+|\d+')
CSV_PATH  = os.getcwd() + "/src/rs485_fire/config/trajectory_angles.csv"


# 物理參數與視覺化解析度（與範例腳本保持一致）
g, v0            = 9.81, 6.5
pitch_range_deg  = (-25, 35)
yaw_range_deg    = (-80, 80)
pitch_vals_full  = np.linspace(*pitch_range_deg, 31)
yaw_vals_full    = np.linspace(*yaw_range_deg, 241)
disp_pitch_deg   = np.linspace(*pitch_range_deg, 13)
disp_yaw_deg     = np.linspace(*yaw_range_deg, 25)
disp_colors      = plt.cm.viridis(np.linspace(0, 1, len(disp_pitch_deg)))
box_margin       = 0.5         # 額外掃描邊緣

# -------- 基礎計算函式 ---------------------------------------------
def nozzle_height(pitch_deg):
    """Nozzle height (m) vs. pitch angle (deg) ‒ 同先前公式"""
    theta = np.radians(pitch_deg)
    return (167 + 22.5*np.sin(theta + np.arcsin(6/22.5))) / 100.0

def xy_at_height(pitch_deg, yaw_deg, target_z):
    """擊中指定平面 Z 時的 (x,y); 失敗回 None"""
    theta, psi = map(np.radians, (pitch_deg, yaw_deg))
    h0 = nozzle_height(pitch_deg)
    a, b, c = -0.5*g, v0*np.sin(theta), h0 - target_z
    disc = b*b - 4*a*c
    if disc < 0:
        return None
    t_hit = (-b - np.sqrt(disc)) / (2*a)
    if t_hit <= 0:
        return None
    x = -v0*np.cos(theta)*np.cos(psi)*t_hit
    y =  v0*np.cos(theta)*np.sin(psi)*t_hit
    return x, y

def xy_landing(pitch_deg, yaw_deg):
    """落地點 (x,y)（z=0）──僅用於稀疏顯示"""
    theta, psi = map(np.radians, (pitch_deg, yaw_deg))
    h0 = nozzle_height(pitch_deg)
    t_land = (v0*np.sin(theta) + np.sqrt((v0*np.sin(theta))**2 + 2*g*h0)) / g
    d = v0*np.cos(theta) * t_land
    return -d*np.cos(psi), d*np.sin(psi)

# ===================================================================
class SprayPlannerNode(Node):

    # -------------------- ROS 初期化 --------------------
    def __init__(self):
        super().__init__("spray_planner")
        self._buf = []
        self._sub_fire = self.create_subscription(
            String, "/fire_info", self._fire_cb, 10)
        self.get_logger().info(f"⏳ 正在收集 /fire_info… ({SAMPLES} 筆)")

        qos = QoSProfile(depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pub_xy  = self.create_publisher(Image , "/plan_xy_image", qos)
        self._pub_xz  = self.create_publisher(Image , "/plan_xz_image", qos)
        self._pub_sta = self.create_publisher(String, "/plan_status" , qos)
        self._bridge  = CvBridge()

    # -------------------- 收集 fire_info -----------------
    def _fire_cb(self, msg: String):
        # 把字串裡所有數字抓出來
        nums = [float(x) for x in DATA_RE.findall(msg.data)]

        # 至少要有 6 個數字（center 3 + size 3）
        if len(nums) < 6:
            return

        # 取最後 6 個數字作為 (cx, cy, cz, lx, ly, lz)
        cx, cy, cz, lx, ly, lz = nums[-6:]

        self._buf.append((cx, cy, cz, lx, ly, lz))
        self.get_logger().info(f"[{len(self._buf)}/{SAMPLES}] fire_info parsed "
                            f"-> center=({cx:.2f},{cy:.2f},{cz:.2f}) "
                            f"size=({lx:.2f},{ly:.2f},{lz:.2f})")

        if len(self._buf) < SAMPLES:
            return

        # 收滿就開始規劃
        self.destroy_subscription(self._sub_fire)
        self._sub_fire = None
        self.get_logger().info("✔ 收集完成，開始路徑規劃…")
        threading.Thread(target=self._plan_and_publish, daemon=True).start()


    # -------------------- 主流程 -------------------------
    def _plan_and_publish(self):
        # (1) 取最穩定的一筆
        cx, cy, cz, lx, ly, lz = self._pick_stable()
        # cx, cy, cz, lx, ly, lz = 3.2, 0, 0, 0.4, 0.4, 0.4
        plane_z = cz  # 使用 box 中心平面

        # (2) 計算掃描角
        traj_xy, traj_ang = self._compute_traj(cx, cy, lx, ly, plane_z)

        # (3) 寫 CSV
        self._save_csv(traj_ang)

        # (4) 畫圖（稀疏落地 + 掃描線）
        fig_xy, fig_xz = self._draw_figs(cx, cy, lx, ly, plane_z, traj_xy)

        # (5) 轉 Image Msg
        xy_msg = self._fig_to_msg(fig_xy)
        xz_msg = self._fig_to_msg(fig_xz)
        sta_msg = String(data="finished")

        # (6) 0.5 Hz 持續發佈
        def timer_cb():
            self._pub_xy.publish(xy_msg)
            self._pub_xz.publish(xz_msg)
            self._pub_sta.publish(sta_msg)
        self.create_timer(0.5, timer_cb)
        self.get_logger().info("📣 已開始持續發佈 /plan_* topic（0.5 Hz）")

    # -------------------- 計算輔助 -----------------------
    def _pick_stable(self):
        centres = [(d[0], d[1], d[2]) for d in self._buf]
        avg = tuple(map(statistics.mean, zip(*centres)))
        best = min(self._buf, key=lambda d:
                   sum((ci-ai)**2 for ci,ai in zip(d[:3], avg)))
        cx, cy, cz, lx, ly, lz = best
        if cz<0:
            self.get_logger().warn("⚠️ 水平面 Z < 0，將其視為 0")
            cz = 0.0
        cx, cy, cz = -cx, -(cy+0.1), cz                        # 座標系轉換
        self.get_logger().info(
            f"  ▸ center=({cx:.2f},{cy:.2f},{cz:.2f}) "
            f"size=({lx:.2f},{ly:.2f},{lz:.2f})")
        return cx, cy, cz, lx, ly, lz

    def _compute_traj(self, cx, cy, lx, ly, plane_z):
        x_min, x_max = cx - lx/2 - box_margin, cx + lx/2 + box_margin
        y_min, y_max = cy - ly/2 - box_margin, cy + ly/2 + box_margin

        traj_xy, traj_ang, zig = [], [], True
        for p in pitch_vals_full:
            yaws = yaw_vals_full if zig else yaw_vals_full[::-1]
            for y in yaws:
                hit = xy_at_height(p, y, plane_z)
                if hit and x_min <= hit[0] <= x_max and y_min <= hit[1] <= y_max:
                    traj_xy.append(hit)
                    traj_ang.append((p, y))
            zig = not zig
        self.get_logger().info(f"  ▸ 命中角度 {len(traj_ang)} 組")
        return traj_xy, traj_ang

    # -------------------- 輸出 CSV -----------------------
    def _save_csv(self, rows):
        with open(CSV_PATH, "w", newline="") as f:
            w = csv.writer(f); w.writerow(["pitch_deg","yaw_deg"]); w.writerows(rows)
        self.get_logger().info(f"  ▸ CSV 已寫入 {CSV_PATH}")

    # -------------------- 畫 XY / XZ ---------------------
    def _draw_figs(self, cx, cy, lx, ly, plane_z, traj_xy):
        # --- XY ---
        fig_xy, ax_xy = plt.subplots(figsize=(8,6), facecolor='none')
        ax_xy.set_facecolor('none')
        # 稀疏彩色落地點
        for c, p in zip(disp_colors, disp_pitch_deg):
            xs, ys = zip(*(xy_landing(p, y) for y in disp_yaw_deg))
            ax_xy.scatter(xs, ys, s=10, color=c, zorder=1)
        # 藍線掃描
        if traj_xy:
            ax_xy.plot(*zip(*traj_xy), c='deepskyblue', lw=2, zorder=2)
        # box / nozzle
        ax_xy.add_patch(plt.Rectangle((cx-lx/2, cy-ly/2), lx, ly,
                                      ec='firebrick', fc='none', lw=2, ls='--'))
        ax_xy.scatter(0,0,c='blue',marker='x',s=70)
        ax_xy.set_title(f"XY plane @ z={plane_z:.2f} m  (sparse view)")
        ax_xy.set_xlabel("x (m)"); ax_xy.set_ylabel("y (m)")
        ax_xy.set_xlim(-7,1); ax_xy.set_ylim(-6,6)
        ax_xy.set_aspect('equal'); ax_xy.grid(True)
        plt.tight_layout()

        # --- XZ ---
        fig_xz, ax_xz = plt.subplots(figsize=(8,4), facecolor='none')
        ax_xz.set_facecolor('none')
        for c, p in zip(disp_colors, disp_pitch_deg):
            th = np.radians(p); h0 = nozzle_height(p)
            t_land = (v0*np.sin(th) + np.sqrt((v0*np.sin(th))**2 + 2*g*h0)) / g
            t = np.linspace(0, t_land, 140)
            x = -v0*np.cos(th)*t
            z =  h0 + v0*np.sin(th)*t - 0.5*g*t**2
            ax_xz.plot(x, z, color=c, lw=0.7)
        if traj_xy:
            ax_xz.scatter([pt[0] for pt in traj_xy], [plane_z]*len(traj_xy),
                          c='deepskyblue', s=35, zorder=3)
        ax_xz.axhline(plane_z, color='k', ls='--', lw=1, alpha=0.5)
        ax_xz.scatter(0, nozzle_height(0), c='blue', marker='x', s=70)
        # box 投影
        ax_xz.plot([cx-lx/2-box_margin, cx+lx/2+box_margin,
                    cx+lx/2+box_margin, cx-lx/2-box_margin,
                    cx-lx/2-box_margin],
                   [plane_z]*5, 'firebrick', lw=2, ls='--')
        ax_xz.set_title("Side view (X-Z)")
        ax_xz.set_xlabel("x (m)"); ax_xz.set_ylabel("z (m)")
        ax_xz.set_xlim(-7,1); ax_xz.set_ylim(0, max(plane_z+1,3))
        ax_xz.set_aspect('equal'); ax_xz.grid(True)
        plt.tight_layout()

        # # ---- 桌面預覽（非阻塞）----
        # fig_xy.show(); fig_xz.show(); plt.pause(0.001)
        return fig_xy, fig_xz

    # -------------------- 轉 ROS Image -------------------
    def _fig_to_msg(self, fig):
        canvas = FigureCanvasAgg(fig); canvas.draw()
        w,h = canvas.get_width_height()
        buf = np.frombuffer(canvas.tostring_rgb(), dtype=np.uint8).reshape(h,w,3)
        return self._bridge.cv2_to_imgmsg(buf, "rgb8")

# ------------------------------- main ------------------------------
def main():
    rclpy.init()
    node = SprayPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()       # 🔹 建議也補上資源清除
        if rclpy.ok():            # ✅ 多這行可防止已 shutdown 時重複呼叫錯誤
            rclpy.shutdown()


if __name__ == "__main__":
    main()
