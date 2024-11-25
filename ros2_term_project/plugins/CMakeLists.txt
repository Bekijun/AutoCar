cmake_minimum_required(VERSION 3.12)
project(MovingBoxPlugin)
# Gazebo 및 기타 종속성 찾기
find_package(gazebo 11 REQUIRED)
# 플러그인 소스 코드 빌드
add_library(MovingBoxPlugin SHARED MovingBoxPlugin.cpp)
# Gazebo 헤더 파일 포함
target_include_directories(MovingBoxPlugin PRIVATE ${GAZEBO_INCLUDE_DIRS})
# Gazebo 라이브러리와 링크
target_link_libraries(MovingBoxPlugin ${GAZEBO_LIBRARIES})
