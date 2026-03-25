cmake_minimum_required(VERSION 2.8.3)
project(mpu_6050_driver)

find_package(catkin REQUIRED COMPONENTS rospy sensor_msgs)


# find_package(Boost REQUIRED COMPONENTS system)


catkin_python_setup()


# add_message_files(

# add_service_files(

# add_action_files(

# generate_messages(

catkin_package()


# include_directories(include)

# add_library(mpu_6050_driver
#   src/${PROJECT_NAME}/mpu_6050_driver.cpp
# )

# add_executable(mpu_6050_driver_node src/mpu_6050_driver_node.cpp)


# add_dependencies(mpu_6050_driver_node mpu_6050_driver_generate_messages_cpp)

# target_link_libraries(mpu_6050_driver_node
#   ${catkin_LIBRARIES}
# )


# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(TARGETS mpu_6050_driver mpu_6050_driver_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )


# catkin_add_gtest(${PROJECT_NAME}-test test/test_mpu_6050_driver.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

# catkin_add_nosetests(test)
