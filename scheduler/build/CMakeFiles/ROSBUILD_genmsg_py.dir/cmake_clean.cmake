FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/scheduler/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/scheduler/msg/__init__.py"
  "../src/scheduler/msg/_SchedulerAction.py"
  "../src/scheduler/msg/_SchedulerGoal.py"
  "../src/scheduler/msg/_SchedulerActionGoal.py"
  "../src/scheduler/msg/_SchedulerResult.py"
  "../src/scheduler/msg/_SchedulerActionResult.py"
  "../src/scheduler/msg/_SchedulerFeedback.py"
  "../src/scheduler/msg/_SchedulerActionFeedback.py"
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
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
