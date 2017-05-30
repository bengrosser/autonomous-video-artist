; Auto-generated. Do not edit!


(cl:in-package im_msgs-msg)


;//! \htmlinclude Bumper.msg.html

(cl:defclass <Bumper> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (state
    :reader state
    :initarg :state
    :type (cl:vector im_msgs-msg:BumperState)
   :initform (cl:make-array 0 :element-type 'im_msgs-msg:BumperState :initial-element (cl:make-instance 'im_msgs-msg:BumperState))))
)

(cl:defclass Bumper (<Bumper>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Bumper>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Bumper)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name im_msgs-msg:<Bumper> is deprecated: use im_msgs-msg:Bumper instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Bumper>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader im_msgs-msg:header-val is deprecated.  Use im_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <Bumper>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader im_msgs-msg:state-val is deprecated.  Use im_msgs-msg:state instead.")
  (state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Bumper>) ostream)
  "Serializes a message object of type '<Bumper>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'state))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'state))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Bumper>) istream)
  "Deserializes a message object of type '<Bumper>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'state) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'state)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'im_msgs-msg:BumperState))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Bumper>)))
  "Returns string type for a message object of type '<Bumper>"
  "im_msgs/Bumper")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Bumper)))
  "Returns string type for a message object of type 'Bumper"
  "im_msgs/Bumper")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Bumper>)))
  "Returns md5sum for a message object of type '<Bumper>"
  "7922bf6f8c5052b10cab99bba5f8096d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Bumper)))
  "Returns md5sum for a message object of type 'Bumper"
  "7922bf6f8c5052b10cab99bba5f8096d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Bumper>)))
  "Returns full string definition for message of type '<Bumper>"
  (cl:format cl:nil "std_msgs/Header header~%im_msgs/BumperState[] state~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: im_msgs/BumperState~%bool bumper_state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Bumper)))
  "Returns full string definition for message of type 'Bumper"
  (cl:format cl:nil "std_msgs/Header header~%im_msgs/BumperState[] state~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: im_msgs/BumperState~%bool bumper_state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Bumper>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'state) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Bumper>))
  "Converts a ROS message object to a list"
  (cl:list 'Bumper
    (cl:cons ':header (header msg))
    (cl:cons ':state (state msg))
))
