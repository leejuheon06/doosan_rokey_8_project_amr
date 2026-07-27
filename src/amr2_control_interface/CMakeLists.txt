cmake_minimum_required(VERSION 3.8)
project(amr2_control_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 의존성 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)


set(msg_files
  "msg/Amr2ControlSignal.msg"
)


# 인터페이스 생성 (DEPENDENCIES 수정됨)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # find_package(amr2_control_interface)
  # set(msg)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()