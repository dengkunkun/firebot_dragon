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
find_package(OpenCV REQUIRED)
find_package(fire_interfaces REQUIRED) 

set(HK_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/HCNetSDKV6.1.9.45_build20220902_ArmLinux64_ZH)
include_directories(${HK_LIB_DIR}/incCn)
# include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../playm4_include)
link_directories(${HK_LIB_DIR}/MakeAll)
link_directories(${HK_LIB_DIR}/MakeAll/HCNetSDKCom)

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
  # 添加你的海康SDK库，例如 -lHCNetSDK
  hcnetsdk
  HCCore
  hpr
  # NPQos
  PlayCtrl
  SuperRender
  z
  crypto
  ssl
  AudioRender
  # openal
  # HCNetSDKCom子目录
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

install(TARGETS hk_camera_pub_jpeg
  DESTINATION lib/${PROJECT_NAME})



  add_executable(hk_camera_pub_decode src/hk_camera_pub_decode.cpp)
  ament_target_dependencies(hk_camera_pub_decode
    rclcpp
    rclcpp_lifecycle
    sensor_msgs
    cv_bridge
    OpenCV
  )
  
  target_link_libraries(hk_camera_pub_decode
    ${OpenCV_LIBS}
    hcnetsdk
    HCCore
    hpr
    # NPQos
    PlayCtrl
    SuperRender
    z
    crypto
    ssl
    AudioRender
    # openal
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
  
  install(TARGETS hk_camera_pub_decode
    DESTINATION lib/${PROJECT_NAME})


add_executable(fireinfo_pub src/fireinfo_pub.cpp)
    ament_target_dependencies(fireinfo_pub
      rclcpp
      rclcpp_lifecycle
      sensor_msgs
      cv_bridge
      OpenCV
      fire_interfaces
    )
    
    target_link_libraries(fireinfo_pub
      ${OpenCV_LIBS}
      hcnetsdk
      HCCore
      hpr
      # NPQos
      PlayCtrl
      SuperRender
      z
      crypto
      ssl
      AudioRender
      # openal
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
    
    install(TARGETS fireinfo_pub
      DESTINATION lib/${PROJECT_NAME})
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