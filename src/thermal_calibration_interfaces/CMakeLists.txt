cmake_minimum_required(VERSION 3.8)
project(thermal_calibration_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CalibrationPoint.msg"
  "msg/CalibrationModel.msg"
  "srv/GetRawValue.srv"
  "srv/AddCalibrationPoint.srv"
  "srv/PerformCalibration.srv"
  "srv/ClearCalibrationData.srv"
  "srv/RawToTemperature.srv"
  "srv/SaveCalibrationModel.srv"
  "srv/LoadCalibrationModel.srv"
  DEPENDENCIES std_msgs sensor_msgs builtin_interfaces
)

ament_package()
