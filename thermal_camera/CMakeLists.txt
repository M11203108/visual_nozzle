cmake_minimum_required(VERSION 3.5)
project(thermal_camera)

# ───── 1.  C++ 編譯選項 ─────────────────────────────
set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wall -Wextra -Wpedantic)

# ───── 2.  Include path ────────────────────────────
include_directories(
  include
  ${PROJECT_SOURCE_DIR}/include/thermal_camera
)

# ───── 3.  ROS 2 & 其他依賴 ─────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# ───── 4.  系統 / 第三方 Library ────────────────────
find_library(UDEV_LIB     udev     REQUIRED)
find_library(PTHREAD_LIB  pthread  REQUIRED)
find_library(M_LIB        m        REQUIRED)

# ───── 5.  C++ 節點 ─────────────────────────────────
add_executable(thermal_camera_node src/ir_camera_node.cpp)
ament_target_dependencies(
  thermal_camera_node
  rclcpp sensor_msgs cv_bridge OpenCV image_transport
)

# GuideUSB2LiveStream 靜態 / 動態庫在 ${PROJECT_SOURCE_DIR}/lib
link_directories(${PROJECT_SOURCE_DIR}/lib)
target_link_libraries(thermal_camera_node
  GuideUSB2LiveStream
  ${UDEV_LIB} ${PTHREAD_LIB} ${M_LIB} ${OpenCV_LIBS}
)

# ───── 6.  安裝 ─────────────────────────────────────
# C++ 可執行檔
install(TARGETS thermal_camera_node
  DESTINATION lib/${PROJECT_NAME})

# Public headers
install(DIRECTORY include/
  DESTINATION include)

# 所有 launch 檔（含 fire_all_launch.py）
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

# Python 節點 & 其他腳本
install(PROGRAMS
  scripts/fire_detection.py
  scripts/thermal_to_rgb_pixel_node.py          # ← 新增的節點
  scripts/camera_calibration_circle.py
  scripts/triangulate_node.py
  scripts/thermal_fusion_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# 權重 / 其他資料檔
install(FILES
  scripts/best.pt
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
