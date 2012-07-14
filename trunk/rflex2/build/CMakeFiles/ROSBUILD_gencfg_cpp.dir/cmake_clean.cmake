FILE(REMOVE_RECURSE
  "CMakeFiles/ROSBUILD_gencfg_cpp"
  "../cfg/cpp/rflex2/B21Config.h"
  "../docs/B21Config.dox"
  "../docs/B21Config-usage.dox"
  "../src/rflex2/cfg/B21Config.py"
  "../docs/B21Config.wikidoc"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gencfg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
