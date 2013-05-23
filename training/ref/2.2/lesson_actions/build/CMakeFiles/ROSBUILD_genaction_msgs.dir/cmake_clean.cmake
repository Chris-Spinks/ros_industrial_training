FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/lesson_actions/msg"
  "CMakeFiles/ROSBUILD_genaction_msgs"
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
  INCLUDE(CMakeFiles/ROSBUILD_genaction_msgs.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
