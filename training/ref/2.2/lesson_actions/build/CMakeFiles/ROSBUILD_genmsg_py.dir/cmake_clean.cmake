FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/lesson_actions/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/lesson_actions/msg/__init__.py"
  "../src/lesson_actions/msg/_FibonacciAction.py"
  "../src/lesson_actions/msg/_FibonacciGoal.py"
  "../src/lesson_actions/msg/_FibonacciActionGoal.py"
  "../src/lesson_actions/msg/_FibonacciResult.py"
  "../src/lesson_actions/msg/_FibonacciActionResult.py"
  "../src/lesson_actions/msg/_FibonacciFeedback.py"
  "../src/lesson_actions/msg/_FibonacciActionFeedback.py"
  "../msg/FibonacciAction.msg"
  "../msg/FibonacciGoal.msg"
  "../msg/FibonacciActionGoal.msg"
  "../msg/FibonacciResult.msg"
  "../msg/FibonacciActionResult.msg"
  "../msg/FibonacciFeedback.msg"
  "../msg/FibonacciActionFeedback.msg"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
