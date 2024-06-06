cmake_minimum_required(VERSION 3.8)
project(testqt)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# qt dependencies
find_package(Qt5 REQUIRED COMPONENTS
  Qml
  Quick
)

file(GLOB_RECURSE SRC_FILE "src/*.cpp")
add_executable(${PROJECT_NAME}
  ${SRC_FILE}
  qml.qrc
)

target_link_libraries(${PROJECT_NAME}
  Qt5::Qml
  Qt5::Quick
)

# include(GNUInstallDirs)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()
