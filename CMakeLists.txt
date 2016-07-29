cmake_minimum_required(VERSION 2.8.3)
project(needybot_speech)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  needybot_msgs 
  rospy
  std_msgs
)

# Run install.sh
execute_process(
  COMMAND
  ${CMAKE_CURRENT_SOURCE_DIR}/install.sh
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    needybot_msgs
    message_runtime
    std_msgs
    rospy
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/needybot_speech.test)
endif()
