cmake_minimum_required(VERSION 3.5)
project(gphoto2_cam)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(OpenCV REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(avcodec REQUIRED libavcodec)
pkg_check_modules(avutil REQUIRED libavutil)
pkg_check_modules(swscale REQUIRED libswscale)

if(EXISTS ${avcodec})
  message(STATUS "Found libavcodec: ${avcodec}")
endif()

if(EXISTS ${avutil})
  message(STATUS "Found libavutil: ${avutil}")
endif()

if(EXISTS ${swscale})
  message(STATUS "Found libswscale: ${swscale}")
endif()

# Find gphoto2
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")
find_package(GPHOTO2 REQUIRED)

find_package(camera_info_manager REQUIRED)

# # Build the USB camera library
# # Do not use ament_auto here so as to not link to rclcpp
add_library(${PROJECT_NAME} SHARED
  src/gphoto2_cam.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  "include"
  ${OpenCV_INCLUDE_DIRS}
  ${LIBGPHOTO2_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME}
  ${avcodec_LIBRARIES}
  ${avutil_LIBRARIES}
  ${swscale_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${LIBGPHOTO2_LIBRARIES})

ament_export_libraries(${PROJECT_NAME})

# # Declare a ROS 2 composible node as a library
ament_auto_add_library(${PROJECT_NAME}_node SHARED
  src/gphoto2_cam_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME})

# # Use node to generate an executable
rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "gphoto2_cam::gPhoto2CamNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)

if(SANITIZE)
  target_compile_options(${PROJECT_NAME} PUBLIC -fsanitize=address -fsanitize=leak)
  target_link_libraries(${PROJECT_NAME} -fsanitize=address -fsanitize=leak)
  target_compile_options(${PROJECT_NAME}_node PUBLIC -fsanitize=address -fsanitize=leak)
  target_link_libraries(${PROJECT_NAME}_node -fsanitize=address -fsanitize=leak)
  target_link_libraries(${PROJECT_NAME}_node_exe -fsanitize=address -fsanitize=leak)
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest)

  # Unit tests
  ament_add_gtest(test_gphoto2_cam_utils
    test/test_gphoto2_cam_utils.cpp)
  target_link_libraries(test_gphoto2_cam_utils
    ${PROJECT_NAME})
  ament_add_gtest(test_pixel_formats
    test/test_pixel_formats.cpp)
  target_link_libraries(test_pixel_formats
    ${PROJECT_NAME})

  # TODO(flynneva): rewrite this test in another PR
  # Integration tests
  # ament_add_gtest(test_gphoto2_cam_lib
  # test/test_gphoto2_cam_lib.cpp)
  # target_link_libraries(test_gphoto2_cam_lib
  # ${PROJECT_NAME})
endif()

install(PROGRAMS
  scripts/show_image.py
  scripts/monitor_node.py
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
