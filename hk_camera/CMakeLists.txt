cmake_minimum_required(VERSION 3.8)
project(hk_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)  
find_package(OpenCV REQUIRED)
find_package(fire_interfaces REQUIRED) 
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# FFmpeg 依赖 - 移到前面
find_package(PkgConfig REQUIRED)
pkg_check_modules(FFMPEG REQUIRED libavcodec libavformat libswscale libavutil)
pkg_check_modules(LIBAV REQUIRED IMPORTED_TARGET libavdevice libavformat libavfilter libavcodec libswresample libswscale libavutil)

set(HK_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/HCNetSDKV6.1.9.45_build20220902_ArmLinux64_ZH)
include_directories(${HK_LIB_DIR}/incCn)
link_directories(${HK_LIB_DIR}/MakeAll)
link_directories(${HK_LIB_DIR}/MakeAll/HCNetSDKCom)

# hk_camera_pub_jpeg (原有的)
add_executable(hk_camera_pub_jpeg src/hk_camera_pub_jpeg.cpp)
ament_target_dependencies(hk_camera_pub_jpeg
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  cv_bridge
  OpenCV
)

target_link_libraries(hk_camera_pub_jpeg
  ${OpenCV_LIBS}
  hcnetsdk
  HCCore
  hpr
  PlayCtrl
  SuperRender
  z
  crypto
  ssl
  AudioRender
  HCAlarm
  HCCoreDevCfg
  HCDisplay
  HCGeneralCfgMgr
  HCIndustry
  HCPlayBack
  HCPreview
  HCVoiceTalk
  iconv2
  StreamTransClient
  SystemTransform
  analyzedata
  AudioIntercom
)

# hk_camera_pub_decode (修复版本)
add_executable(hk_camera_pub_decode src/hk_camera_pub_decode.cpp)
ament_target_dependencies(hk_camera_pub_decode
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  cv_bridge
  OpenCV
)

# 添加FFmpeg头文件路径
target_include_directories(hk_camera_pub_decode PRIVATE 
    ${FFMPEG_INCLUDE_DIRS}
)

# 添加编译器定义和标志
target_compile_definitions(hk_camera_pub_decode PRIVATE 
    ${FFMPEG_CFLAGS_OTHER}
)

target_link_libraries(hk_camera_pub_decode
    ${OpenCV_LIBS}
    ${FFMPEG_LIBRARIES}  # FFmpeg库放在前面
    hcnetsdk
    HCCore
    hpr
    PlayCtrl
    SuperRender
    z
    crypto
    ssl
    AudioRender
    HCAlarm
    HCCoreDevCfg
    HCDisplay
    HCGeneralCfgMgr
    HCIndustry
    HCPlayBack
    HCPreview
    HCVoiceTalk
    iconv2
    StreamTransClient
    SystemTransform
    analyzedata
    AudioIntercom
)

# fireinfo_pub (原有的)
add_executable(fireinfo_pub src/fireinfo_pub.cpp)
ament_target_dependencies(fireinfo_pub
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  cv_bridge
  OpenCV
  fire_interfaces
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(fireinfo_pub
  ${OpenCV_LIBS}
  hcnetsdk
  HCCore
  hpr
  PlayCtrl
  SuperRender
  z
  crypto
  ssl
  AudioRender
  HCAlarm
  HCCoreDevCfg
  HCDisplay
  HCGeneralCfgMgr
  HCIndustry
  HCPlayBack
  HCPreview
  HCVoiceTalk
  iconv2
  StreamTransClient
  SystemTransform
  analyzedata
  AudioIntercom
)
# target_include_directories(fireinfo_pub PRIVATE
#   /opt/ros/jazzy/include/tf2_ros
# )

# 添加可执行文件 - 修正版本
add_executable(hk_camera_ffmpeg_rtsp src/hk_camera_ffmpeg_rtsp.cpp)
ament_target_dependencies(hk_camera_ffmpeg_rtsp 
  rclcpp 
  sensor_msgs 
  cv_bridge 
  image_transport
  OpenCV
)
target_link_libraries(hk_camera_ffmpeg_rtsp PkgConfig::LIBAV)

# 安装目标
install(TARGETS 
  hk_camera_pub_jpeg 
  hk_camera_pub_decode 
  fireinfo_pub
  hk_camera_ffmpeg_rtsp
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()