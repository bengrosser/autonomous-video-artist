; Auto-generated. Do not edit!


(cl:in-package im_msgs-msg)


;//! \htmlinclude BumperState.msg.html

(cl:defclass <BumperState> (roslisp-msg-protocol:ros-message)
  ((bumper_state
    :reader bumper_state
    :initarg :bumper_state
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass BumperState (<BumperState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BumperState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BumperState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name im_msgs-msg:<BumperState> is deprecated: use im_msgs-msg:BumperState instead.")))

(cl:ensure-generic-function 'bumper_state-val :lambda-list '(m))
(cl:defmethod bumper_state-val ((m <BumperState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader im_msgs-msg:bumper_state-val is deprecated.  Use im_msgs-msg:bumper_state instead.")
  (bumper_state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BumperState>) ostream)
  "Serializes a message object of type '<BumperState>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'bumper_state) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BumperState>) istream)
  "Deserializes a message object of type '<BumperState>"
    (cl:setf (cl:slot-value msg 'bumper_state) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BumperState>)))
  "Returns string type for a message object of type '<BumperState>"
  "im_msgs/BumperState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BumperState)))
  "Returns string type for a message object of type 'BumperState"
  "im_msgs/BumperState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BumperState>)))
  "Returns md5sum for a message object of type '<BumperState>"
  "46e92a89f2364b7e5f107580d7840ab7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BumperState)))
  "Returns md5sum for a message object of type 'BumperState"
  "46e92a89f2364b7e5f107580d7840ab7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BumperState>)))
  "Returns full string definition for message of type '<BumperState>"
  (cl:format cl:nil "bool bumper_state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BumperState)))
  "Returns full string definition for message of type 'BumperState"
  (cl:format cl:nil "bool bumper_state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BumperState>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BumperState>))
  "Converts a ROS message object to a list"
  (cl:list 'BumperState
    (cl:cons ':bumper_state (bumper_state msg))
))
