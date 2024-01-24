cmake_minimum_required(VERSION 3.16)
project(SI_CBS)

# C++ 표준 설정
set(CMAKE_CXX_STANDARD 17)

# include 디렉토리 포함
include_directories("include")

# Boost 라이브러리 찾기
find_package(Boost REQUIRED)

# yaml-cpp 라이브러리 경로 설정
#set(YAML_CPP_INCLUDE_DIR "/opt/homebrew/Cellar/yaml-cpp/0.8.0/include")
#set(YAML_CPP_LIBRARIES "/opt/homebrew/Cellar/yaml-cpp/0.8.0/lib/libyaml-cpp.dylib")

# yaml-cpp 라이브러리 찾기 (선택적)
find_package(yaml-cpp REQUIRED)

# Boost 및 yaml-cpp include 디렉토리 포함
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${YAML_CPP_INCLUDE_DIR})

# CMake 모듈 경로에 yaml-cpp 추가 (선택적)
list(APPEND CMAKE_PREFIX_PATH "/opt/homebrew/Cellar/yaml-cpp/0.8.0/lib/cmake/yaml-cpp")

# 소스 파일 정의
file(GLOB SOURCES "src/*.cpp")

# 실행 파일 추가
add_executable(SI-CBS ${SOURCES})

# 라이브러리 링크
target_link_libraries(SI-CBS ${Boost_LIBRARIES})
target_link_libraries(SI-CBS ${YAML_CPP_LIBRARIES})
