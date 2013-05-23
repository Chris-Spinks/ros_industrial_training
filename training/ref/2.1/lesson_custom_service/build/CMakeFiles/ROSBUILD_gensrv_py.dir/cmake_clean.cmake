FILE(REMOVE_RECURSE
  "../srv_gen"
  "../src/lesson_custom_service/srv"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/lesson_custom_service/srv/__init__.py"
  "../src/lesson_custom_service/srv/_AddTwoInts.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
