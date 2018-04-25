# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${DEMO_APPLICATION_BINARY_DIR}/00-kuka_joint_hold_pos)
ADD_EXECUTABLE (orientation_error_test00 orientation_error_test.cpp)
ADD_EXECUTABLE (controller_oscillate00 controller_oscillate.cpp)
ADD_EXECUTABLE (controller_coriolis00 controller_coriolis.cpp)
ADD_EXECUTABLE (controller_operations_space00 controller_operations_space.cpp)
ADD_EXECUTABLE (controller_operations_space_three_tasks00 controller_operations_space_three_tasks.cpp)
ADD_EXECUTABLE (simviz00 simviz.cpp)

# and link the library against the executable
TARGET_LINK_LIBRARIES (orientation_error_test00 ${DEMO_APPLICATION_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (controller_oscillate00 ${DEMO_APPLICATION_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (controller_coriolis00 ${DEMO_APPLICATION_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (controller_operations_space00 ${DEMO_APPLICATION_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (controller_operations_space_three_tasks00 ${DEMO_APPLICATION_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (simviz00 ${DEMO_APPLICATION_COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(RESOURCE_DIR ${DEMO_APPLICATION_BINARY_DIR}/resources)
FILE(MAKE_DIRECTORY ${RESOURCE_DIR})
SET(APP_RESOURCE_DIR ${RESOURCE_DIR}/00-kuka_joint_hold_pos)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY world.urdf iiwa7.urdf DESTINATION ${APP_RESOURCE_DIR})