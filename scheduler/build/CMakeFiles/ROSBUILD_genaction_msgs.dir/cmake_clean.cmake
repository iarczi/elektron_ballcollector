FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/scheduler/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genaction_msgs"
  "../msg/SchedulerAction.msg"
  "../msg/SchedulerGoal.msg"
  "../msg/SchedulerActionGoal.msg"
  "../msg/SchedulerResult.msg"
  "../msg/SchedulerActionResult.msg"
  "../msg/SchedulerFeedback.msg"
  "../msg/SchedulerActionFeedback.msg"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genaction_msgs.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
