FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/scheduler/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/scheduler/SchedulerAction.h"
  "../msg_gen/cpp/include/scheduler/SchedulerGoal.h"
  "../msg_gen/cpp/include/scheduler/SchedulerActionGoal.h"
  "../msg_gen/cpp/include/scheduler/SchedulerResult.h"
  "../msg_gen/cpp/include/scheduler/SchedulerActionResult.h"
  "../msg_gen/cpp/include/scheduler/SchedulerFeedback.h"
  "../msg_gen/cpp/include/scheduler/SchedulerActionFeedback.h"
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
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
