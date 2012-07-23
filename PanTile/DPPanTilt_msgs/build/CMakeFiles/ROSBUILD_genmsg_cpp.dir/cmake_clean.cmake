FILE(REMOVE_RECURSE
  "../src/DPPanTilt_msgs/msg"
  "../msg_gen"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/DPPanTilt_msgs/SetAbsolutePositionPan.h"
  "../msg_gen/cpp/include/DPPanTilt_msgs/Reset.h"
  "../msg_gen/cpp/include/DPPanTilt_msgs/ReportStaticInfo.h"
  "../msg_gen/cpp/include/DPPanTilt_msgs/ReportDynamicInfo.h"
  "../msg_gen/cpp/include/DPPanTilt_msgs/SetSpeedForPositionMovesPan.h"
  "../msg_gen/cpp/include/DPPanTilt_msgs/SetAbsolutePositionTilt.h"
  "../msg_gen/cpp/include/DPPanTilt_msgs/SetRelativePosition.h"
  "../msg_gen/cpp/include/DPPanTilt_msgs/SetVelocityTilt.h"
  "../msg_gen/cpp/include/DPPanTilt_msgs/SetAbsolutePosition.h"
  "../msg_gen/cpp/include/DPPanTilt_msgs/StartPositionControl.h"
  "../msg_gen/cpp/include/DPPanTilt_msgs/SetSpeedForPositionMovesTilt.h"
  "../msg_gen/cpp/include/DPPanTilt_msgs/StartVelocityControl.h"
  "../msg_gen/cpp/include/DPPanTilt_msgs/HaltMotion.h"
  "../msg_gen/cpp/include/DPPanTilt_msgs/SetVelocityPan.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
