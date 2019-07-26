cmake_minimum_required(VERSION 2.8.3)
project(speech_cmd)

find_package(catkin REQUIRED)
catkin_package(DEPENDS)

install(DIRECTORY demo
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(PROGRAMS nodes/cmd.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
