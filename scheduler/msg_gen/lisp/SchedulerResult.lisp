; Auto-generated. Do not edit!


(cl:in-package scheduler-msg)


;//! \htmlinclude SchedulerResult.msg.html

(cl:defclass <SchedulerResult> (roslisp-msg-protocol:ros-message)
  ((value
    :reader value
    :initarg :value
    :type cl:integer
    :initform 0))
)

(cl:defclass SchedulerResult (<SchedulerResult>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SchedulerResult>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SchedulerResult)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name scheduler-msg:<SchedulerResult> is deprecated: use scheduler-msg:SchedulerResult instead.")))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <SchedulerResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader scheduler-msg:value-val is deprecated.  Use scheduler-msg:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SchedulerResult>) ostream)
  "Serializes a message object of type '<SchedulerResult>"
  (cl:let* ((signed (cl:slot-value msg 'value)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SchedulerResult>) istream)
  "Deserializes a message object of type '<SchedulerResult>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'value) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SchedulerResult>)))
  "Returns string type for a message object of type '<SchedulerResult>"
  "scheduler/SchedulerResult")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SchedulerResult)))
  "Returns string type for a message object of type 'SchedulerResult"
  "scheduler/SchedulerResult")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SchedulerResult>)))
  "Returns md5sum for a message object of type '<SchedulerResult>"
  "b3087778e93fcd34cc8d65bc54e850d1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SchedulerResult)))
  "Returns md5sum for a message object of type 'SchedulerResult"
  "b3087778e93fcd34cc8d65bc54e850d1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SchedulerResult>)))
  "Returns full string definition for message of type '<SchedulerResult>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#result definition~%int32 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SchedulerResult)))
  "Returns full string definition for message of type 'SchedulerResult"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#result definition~%int32 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SchedulerResult>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SchedulerResult>))
  "Converts a ROS message object to a list"
  (cl:list 'SchedulerResult
    (cl:cons ':value (value msg))
))