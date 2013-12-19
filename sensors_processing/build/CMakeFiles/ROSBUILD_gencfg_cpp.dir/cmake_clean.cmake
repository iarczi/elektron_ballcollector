FILE(REMOVE_RECURSE
  "CMakeFiles/ROSBUILD_gencfg_cpp"
  "../cfg/cpp/sensors_processing/TutorialsConfig.h"
  "../docs/TutorialsConfig.dox"
  "../docs/TutorialsConfig-usage.dox"
  "../src/sensors_processing/cfg/TutorialsConfig.py"
  "../docs/TutorialsConfig.wikidoc"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gencfg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
