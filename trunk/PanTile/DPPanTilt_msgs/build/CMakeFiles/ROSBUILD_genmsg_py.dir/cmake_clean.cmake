FILE(REMOVE_RECURSE
  "../src/DPPanTilt_msgs/msg"
  "../msg_gen"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/DPPanTilt_msgs/msg/__init__.py"
  "../src/DPPanTilt_msgs/msg/_SetAbsolutePositionPan.py"
  "../src/DPPanTilt_msgs/msg/_Reset.py"
  "../src/DPPanTilt_msgs/msg/_ReportStaticInfo.py"
  "../src/DPPanTilt_msgs/msg/_ReportDynamicInfo.py"
  "../src/DPPanTilt_msgs/msg/_SetSpeedForPositionMovesPan.py"
  "../src/DPPanTilt_msgs/msg/_SetAbsolutePositionTilt.py"
  "../src/DPPanTilt_msgs/msg/_SetRelativePosition.py"
  "../src/DPPanTilt_msgs/msg/_SetVelocityTilt.py"
  "../src/DPPanTilt_msgs/msg/_SetAbsolutePosition.py"
  "../src/DPPanTilt_msgs/msg/_StartPositionControl.py"
  "../src/DPPanTilt_msgs/msg/_SetSpeedForPositionMovesTilt.py"
  "../src/DPPanTilt_msgs/msg/_StartVelocityControl.py"
  "../src/DPPanTilt_msgs/msg/_HaltMotion.py"
  "../src/DPPanTilt_msgs/msg/_SetVelocityPan.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
