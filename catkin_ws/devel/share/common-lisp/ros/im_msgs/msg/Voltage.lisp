; Auto-generated. Do not edit!


(cl:in-package im_msgs-msg)


;//! \htmlinclude Voltage.msg.html

(cl:defclass <Voltage> (roslisp-msg-protocol:ros-message)
  ((left_motor
    :reader left_motor
    :initarg :left_motor
    :type cl:float
    :initform 0.0)
   (right_motor
    :reader right_motor
    :initarg :right_motor
    :type cl:float
    :initform 0.0))
)

(cl:defclass Voltage (<Voltage>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Voltage>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Voltage)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name im_msgs-msg:<Voltage> is deprecated: use im_msgs-msg:Voltage instead.")))

(cl:ensure-generic-function 'left_motor-val :lambda-list '(m))
(cl:defmethod left_motor-val ((m <Voltage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader im_msgs-msg:left_motor-val is deprecated.  Use im_msgs-msg:left_motor instead.")
  (left_motor m))

(cl:ensure-generic-function 'right_motor-val :lambda-list '(m))
(cl:defmethod right_motor-val ((m <Voltage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader im_msgs-msg:right_motor-val is deprecated.  Use im_msgs-msg:right_motor instead.")
  (right_motor m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Voltage>) ostream)
  "Serializes a message object of type '<Voltage>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'left_motor))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'right_motor))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Voltage>) istream)
  "Deserializes a message object of type '<Voltage>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'left_motor) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right_motor) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Voltage>)))
  "Returns string type for a message object of type '<Voltage>"
  "im_msgs/Voltage")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Voltage)))
  "Returns string type for a message object of type 'Voltage"
  "im_msgs/Voltage")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Voltage>)))
  "Returns md5sum for a message object of type '<Voltage>"
  "3e3717ac8e9443aa62d7102a5860f5e7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Voltage)))
  "Returns md5sum for a message object of type 'Voltage"
  "3e3717ac8e9443aa62d7102a5860f5e7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Voltage>)))
  "Returns full string definition for message of type '<Voltage>"
  (cl:format cl:nil "float32 left_motor~%float32 right_motor~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Voltage)))
  "Returns full string definition for message of type 'Voltage"
  (cl:format cl:nil "float32 left_motor~%float32 right_motor~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Voltage>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Voltage>))
  "Converts a ROS message object to a list"
  (cl:list 'Voltage
    (cl:cons ':left_motor (left_motor msg))
    (cl:cons ':right_motor (right_motor msg))
))
