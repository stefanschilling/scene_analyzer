FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/scene_analyzer/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/scene_analyzer/msg/__init__.py"
  "../src/scene_analyzer/msg/_stamped_string.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
