FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/lesson_actions/msg"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/lesson_actions/FibonacciAction.h"
  "../msg_gen/cpp/include/lesson_actions/FibonacciGoal.h"
  "../msg_gen/cpp/include/lesson_actions/FibonacciActionGoal.h"
  "../msg_gen/cpp/include/lesson_actions/FibonacciResult.h"
  "../msg_gen/cpp/include/lesson_actions/FibonacciActionResult.h"
  "../msg_gen/cpp/include/lesson_actions/FibonacciFeedback.h"
  "../msg_gen/cpp/include/lesson_actions/FibonacciActionFeedback.h"
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
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
