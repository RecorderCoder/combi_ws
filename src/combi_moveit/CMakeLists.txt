cmake_minimum_required(VERSION 3.22)
project(combi_moveit)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
# launch 폴더의 모든 .py 파일을 설치
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
  
ament_package()

#install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
