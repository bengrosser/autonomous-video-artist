; Auto-generated. Do not edit!


(cl:in-package im_msgs-msg)


;//! \htmlinclude WheelVel.msg.html

(cl:defclass <WheelVel> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (right_vel
    :reader right_vel
    :initarg :right_vel
    :type cl:float
    :initform 0.0)
   (left_vel
    :reader left_vel
    :initarg :left_vel
    :type cl:float
    :initform 0.0))
)

(cl:defclass WheelVel (<WheelVel>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WheelVel>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WheelVel)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name im_msgs-msg:<WheelVel> is deprecated: use im_msgs-msg:WheelVel instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <WheelVel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader im_msgs-msg:header-val is deprecated.  Use im_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'right_vel-val :lambda-list '(m))
(cl:defmethod right_vel-val ((m <WheelVel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader im_msgs-msg:right_vel-val is deprecated.  Use im_msgs-msg:right_vel instead.")
  (right_vel m))

(cl:ensure-generic-function 'left_vel-val :lambda-list '(m))
(cl:defmethod left_vel-val ((m <WheelVel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader im_msgs-msg:left_vel-val is deprecated.  Use im_msgs-msg:left_vel instead.")
  (left_vel m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WheelVel>) ostream)
  "Serializes a message object of type '<WheelVel>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'right_vel))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'left_vel))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WheelVel>) istream)
  "Deserializes a message object of type '<WheelVel>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right_vel) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'left_vel) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WheelVel>)))
  "Returns string type for a message object of type '<WheelVel>"
  "im_msgs/WheelVel")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WheelVel)))
  "Returns string type for a message object of type 'WheelVel"
  "im_msgs/WheelVel")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WheelVel>)))
  "Returns md5sum for a message object of type '<WheelVel>"
  "27fca45ced2bdd7ad9603d4604251411")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WheelVel)))
  "Returns md5sum for a message object of type 'WheelVel"
  "27fca45ced2bdd7ad9603d4604251411")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WheelVel>)))
  "Returns full string definition for message of type '<WheelVel>"
  (cl:format cl:nil "std_msgs/Header header~%float32 right_vel~%float32 left_vel~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WheelVel)))
  "Returns full string definition for message of type 'WheelVel"
  (cl:format cl:nil "std_msgs/Header header~%float32 right_vel~%float32 left_vel~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WheelVel>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WheelVel>))
  "Converts a ROS message object to a list"
  (cl:list 'WheelVel
    (cl:cons ':header (header msg))
    (cl:cons ':right_vel (right_vel msg))
    (cl:cons ':left_vel (left_vel msg))
))
