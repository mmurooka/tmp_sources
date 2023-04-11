set(controller_SRC
  FrameVisualizationController.cpp
)

set(controller_HDR
  FrameVisualizationController.h
)

add_controller(${PROJECT_NAME} ${controller_SRC} ${controller_HDR})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DFrameVisualizationController_EXPORTS")

