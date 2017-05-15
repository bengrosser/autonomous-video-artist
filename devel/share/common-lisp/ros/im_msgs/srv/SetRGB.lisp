; Auto-generated. Do not edit!


(cl:in-package im_msgs-srv)


;//! \htmlinclude SetRGB-request.msg.html

(cl:defclass <SetRGB-request> (roslisp-msg-protocol:ros-message)
  ((mode
    :reader mode
    :initarg :mode
    :type cl:fixnum
    :initform 0)
   (frequency
    :reader frequency
    :initarg :frequency
    :type cl:float
    :initform 0.0)
   (times
    :reader times
    :initarg :times
    :type cl:fixnum
    :initform 0)
   (color
    :reader color
    :initarg :color
    :type cl:fixnum
    :initform 0))
)

(cl:defclass SetRGB-request (<SetRGB-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetRGB-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetRGB-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name im_msgs-srv:<SetRGB-request> is deprecated: use im_msgs-srv:SetRGB-request instead.")))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <SetRGB-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader im_msgs-srv:mode-val is deprecated.  Use im_msgs-srv:mode instead.")
  (mode m))

(cl:ensure-generic-function 'frequency-val :lambda-list '(m))
(cl:defmethod frequency-val ((m <SetRGB-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader im_msgs-srv:frequency-val is deprecated.  Use im_msgs-srv:frequency instead.")
  (frequency m))

(cl:ensure-generic-function 'times-val :lambda-list '(m))
(cl:defmethod times-val ((m <SetRGB-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader im_msgs-srv:times-val is deprecated.  Use im_msgs-srv:times instead.")
  (times m))

(cl:ensure-generic-function 'color-val :lambda-list '(m))
(cl:defmethod color-val ((m <SetRGB-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader im_msgs-srv:color-val is deprecated.  Use im_msgs-srv:color instead.")
  (color m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<SetRGB-request>)))
    "Constants for message type '<SetRGB-request>"
  '((:LEDOFF . 0)
    (:RED . 1)
    (:GREEN . 2)
    (:YELLOW . 3)
    (:BLUE . 4)
    (:PINK . 5)
    (:TURQUOISE . 6)
    (:WHITE . 7)
    (:MANUAL . 0)
    (:ON . 1)
    (:AUTONOMOUS . 2)
    (:TELEOP . 3)
    (:WANDER . 4)
    (:LOW_BATTERY . 5)
    (:CHARGING . 6)
    (:CHARGED . 7)
    (:ERROR . 8))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'SetRGB-request)))
    "Constants for message type 'SetRGB-request"
  '((:LEDOFF . 0)
    (:RED . 1)
    (:GREEN . 2)
    (:YELLOW . 3)
    (:BLUE . 4)
    (:PINK . 5)
    (:TURQUOISE . 6)
    (:WHITE . 7)
    (:MANUAL . 0)
    (:ON . 1)
    (:AUTONOMOUS . 2)
    (:TELEOP . 3)
    (:WANDER . 4)
    (:LOW_BATTERY . 5)
    (:CHARGING . 6)
    (:CHARGED . 7)
    (:ERROR . 8))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetRGB-request>) ostream)
  "Serializes a message object of type '<SetRGB-request>"
  (cl:let* ((signed (cl:slot-value msg 'mode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'frequency))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'times)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'color)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetRGB-request>) istream)
  "Deserializes a message object of type '<SetRGB-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mode) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'frequency) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'times) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'color) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetRGB-request>)))
  "Returns string type for a service object of type '<SetRGB-request>"
  "im_msgs/SetRGBRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetRGB-request)))
  "Returns string type for a service object of type 'SetRGB-request"
  "im_msgs/SetRGBRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetRGB-request>)))
  "Returns md5sum for a message object of type '<SetRGB-request>"
  "09e0749c01af9e427f1a861c70df57ff")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetRGB-request)))
  "Returns md5sum for a message object of type 'SetRGB-request"
  "09e0749c01af9e427f1a861c70df57ff")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetRGB-request>)))
  "Returns full string definition for message of type '<SetRGB-request>"
  (cl:format cl:nil "~%int8 LEDOFF    = 0~%int8 RED       = 1~%int8 GREEN     = 2~%int8 YELLOW    = 3~%int8 BLUE      = 4~%int8 PINK      = 5~%int8 TURQUOISE = 6~%int8 WHITE     = 7~%~%~%int8 MANUAL      = 0~%int8 ON          = 1~%int8 AUTONOMOUS  = 2~%int8 TELEOP      = 3~%int8 WANDER      = 4~%int8 LOW_BATTERY = 5~%int8 CHARGING    = 6~%int8 CHARGED     = 7~%int8 ERROR       = 8~%~%int8 mode~%~%float32 frequency~%~%int8 times~%int8 color~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetRGB-request)))
  "Returns full string definition for message of type 'SetRGB-request"
  (cl:format cl:nil "~%int8 LEDOFF    = 0~%int8 RED       = 1~%int8 GREEN     = 2~%int8 YELLOW    = 3~%int8 BLUE      = 4~%int8 PINK      = 5~%int8 TURQUOISE = 6~%int8 WHITE     = 7~%~%~%int8 MANUAL      = 0~%int8 ON          = 1~%int8 AUTONOMOUS  = 2~%int8 TELEOP      = 3~%int8 WANDER      = 4~%int8 LOW_BATTERY = 5~%int8 CHARGING    = 6~%int8 CHARGED     = 7~%int8 ERROR       = 8~%~%int8 mode~%~%float32 frequency~%~%int8 times~%int8 color~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetRGB-request>))
  (cl:+ 0
     1
     4
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetRGB-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetRGB-request
    (cl:cons ':mode (mode msg))
    (cl:cons ':frequency (frequency msg))
    (cl:cons ':times (times msg))
    (cl:cons ':color (color msg))
))
;//! \htmlinclude SetRGB-response.msg.html

(cl:defclass <SetRGB-response> (roslisp-msg-protocol:ros-message)
  ((ret
    :reader ret
    :initarg :ret
    :type cl:fixnum
    :initform 0))
)

(cl:defclass SetRGB-response (<SetRGB-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetRGB-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetRGB-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name im_msgs-srv:<SetRGB-response> is deprecated: use im_msgs-srv:SetRGB-response instead.")))

(cl:ensure-generic-function 'ret-val :lambda-list '(m))
(cl:defmethod ret-val ((m <SetRGB-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader im_msgs-srv:ret-val is deprecated.  Use im_msgs-srv:ret instead.")
  (ret m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetRGB-response>) ostream)
  "Serializes a message object of type '<SetRGB-response>"
  (cl:let* ((signed (cl:slot-value msg 'ret)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetRGB-response>) istream)
  "Deserializes a message object of type '<SetRGB-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ret) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetRGB-response>)))
  "Returns string type for a service object of type '<SetRGB-response>"
  "im_msgs/SetRGBResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetRGB-response)))
  "Returns string type for a service object of type 'SetRGB-response"
  "im_msgs/SetRGBResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetRGB-response>)))
  "Returns md5sum for a message object of type '<SetRGB-response>"
  "09e0749c01af9e427f1a861c70df57ff")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetRGB-response)))
  "Returns md5sum for a message object of type 'SetRGB-response"
  "09e0749c01af9e427f1a861c70df57ff")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetRGB-response>)))
  "Returns full string definition for message of type '<SetRGB-response>"
  (cl:format cl:nil "int8 ret~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetRGB-response)))
  "Returns full string definition for message of type 'SetRGB-response"
  (cl:format cl:nil "int8 ret~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetRGB-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetRGB-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetRGB-response
    (cl:cons ':ret (ret msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetRGB)))
  'SetRGB-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetRGB)))
  'SetRGB-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetRGB)))
  "Returns string type for a service object of type '<SetRGB>"
  "im_msgs/SetRGB")