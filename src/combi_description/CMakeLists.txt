cmake_minimum_required(VERSION 3.10)
project(combi_description)

find_package(ament_cmake REQUIRED)

# (옵션) xacro가 설치되어 있는지 확인만 필요하면 아래 줄은 생략 가능
find_package(xacro QUIET)

# 리소스 설치
install(DIRECTORY urdf meshes DESTINATION share/${PROJECT_NAME}
)

ament_package()
