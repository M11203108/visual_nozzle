cmake_minimum_required(VERSION 3.8)
project(rs485_fire)

# ──────────────────────────────── 1. 編譯器（保留警告） ────────────────────────────────
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ──────────────────────────────── 2. 依賴套件 (Python-only) ───────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclpy        REQUIRED)
find_package(std_msgs     REQUIRED)
find_package(sensor_msgs  REQUIRED)

# ──────────────────────────────── 3. 安裝 Python 可執行檔 ─────────────────────────────
#  ⬇︎ 確定下列所有 *.py 已 `chmod +x`
install(PROGRAMS
  src/MIX_test/trajectories/spray_trajectories_final_node.py     # 發佈 /plan_* topic
  src/MIX_test/spray_executor_node.py                            # 監聽 /plan_status → 噴灑
  src/MIX_test/spary_motor_pump.py                               # 監聽 /fire_info → 控制噴水馬達
  DESTINATION lib/${PROJECT_NAME}
)

# ── 3-a motor_control 子模組（保持 import motor_control.xxx 能用） ──────────────────
install(DIRECTORY
  src/MIX_test/motor_control
  DESTINATION lib/${PROJECT_NAME}
)

# ── 3-b 校正 JSON（跟腳本放同一層，程式用相對路徑即可找到） ─────────────────────────
install(FILES
  src/MIX_test/calibration_updo.json
  src/MIX_test/calibration_LR.json
  DESTINATION lib/${PROJECT_NAME}
)

# ── 3-c trajectories 目錄（含 trajectory_angles.csv 與其他腳本） ────────────────────
install(DIRECTORY
  src/MIX_test/trajectories
  DESTINATION lib/${PROJECT_NAME}
)

# ──────────────────────────────── 4. 安裝 launch 檔 ──────────────────────────────────
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS
  src/shutdown_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  src/shutdown_trigger_node.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  src/alignment_controller_node.py
  DESTINATION lib/${PROJECT_NAME}
)
# ──────────────────────────────── 5. 測試（維持預設行為） ────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)   # ↖︎ 若有版權檢查可移除
  set(ament_cmake_cpplint_FOUND TRUE)     # ↖︎ 若有 cpplint 可移除
  ament_lint_auto_find_test_dependencies()
endif()

# ──────────────────────────────── 6. 匯出套件 ────────────────────────────────────────
ament_package()
