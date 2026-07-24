cmake_minimum_required(VERSION 3.0.2)
project(zhihang_adaptive_search_v6)
find_package(catkin REQUIRED COMPONENTS
  rospy roslib std_msgs geometry_msgs sensor_msgs gazebo_msgs mavros_msgs cv_bridge tf
)
catkin_python_setup()
catkin_package()
catkin_install_python(PROGRAMS
  scripts/mission_manager.py
  scripts/vehicle_flight_agent.py
  scripts/vehicle_perception_agent.py
  scripts/yolo26_single_worker.py
  scripts/validation_target_state_relay.py
  scripts/vision_target_state_estimator.py
  scripts/competition_result_publisher.py
  scripts/summarize_adaptive_run.py
  scripts/summarize_target_localization.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
