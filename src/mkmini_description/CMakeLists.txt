cmake_minimum_required(VERSION 3.8)
project(mkmini_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config urdf docs launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  ament_add_test(test_mkmini_profile_static
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND ${Python3_EXECUTABLE} -m pytest -q ${CMAKE_CURRENT_SOURCE_DIR}/test/test_mkmini_profile_static.py)
  ament_add_test(test_mk2c1r3_frame_authority
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND ${Python3_EXECUTABLE} -m pytest -q ${CMAKE_CURRENT_SOURCE_DIR}/test/test_mk1d_tf_authority.py)
  set_tests_properties(test_mkmini_profile_static test_mk2c1r3_frame_authority
    PROPERTIES ENVIRONMENT "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1")
endif()

ament_package()
