
(cl:in-package :asdf)

(defsystem "scheduler-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :actionlib_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "SchedulerActionGoal" :depends-on ("_package_SchedulerActionGoal"))
    (:file "_package_SchedulerActionGoal" :depends-on ("_package"))
    (:file "SchedulerAction" :depends-on ("_package_SchedulerAction"))
    (:file "_package_SchedulerAction" :depends-on ("_package"))
    (:file "SchedulerActionFeedback" :depends-on ("_package_SchedulerActionFeedback"))
    (:file "_package_SchedulerActionFeedback" :depends-on ("_package"))
    (:file "SchedulerActionResult" :depends-on ("_package_SchedulerActionResult"))
    (:file "_package_SchedulerActionResult" :depends-on ("_package"))
    (:file "SchedulerResult" :depends-on ("_package_SchedulerResult"))
    (:file "_package_SchedulerResult" :depends-on ("_package"))
    (:file "SchedulerGoal" :depends-on ("_package_SchedulerGoal"))
    (:file "_package_SchedulerGoal" :depends-on ("_package"))
    (:file "SchedulerFeedback" :depends-on ("_package_SchedulerFeedback"))
    (:file "_package_SchedulerFeedback" :depends-on ("_package"))
  ))