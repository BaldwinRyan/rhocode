FILE(REMOVE_RECURSE
  "CMakeFiles/ROSBUILD_gencfg_cpp"
  "../cfg/cpp/pgr_camera_driver/PGRCameraConfig.h"
  "../docs/PGRCameraConfig.dox"
  "../docs/PGRCameraConfig-usage.dox"
  "../src/pgr_camera_driver/cfg/PGRCameraConfig.py"
  "../docs/PGRCameraConfig.wikidoc"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gencfg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
