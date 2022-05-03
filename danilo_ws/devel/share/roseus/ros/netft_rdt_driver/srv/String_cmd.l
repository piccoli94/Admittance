;; Auto-generated. Do not edit!


(when (boundp 'netft_rdt_driver::String_cmd)
  (if (not (find-package "NETFT_RDT_DRIVER"))
    (make-package "NETFT_RDT_DRIVER"))
  (shadow 'String_cmd (find-package "NETFT_RDT_DRIVER")))
(unless (find-package "NETFT_RDT_DRIVER::STRING_CMD")
  (make-package "NETFT_RDT_DRIVER::STRING_CMD"))
(unless (find-package "NETFT_RDT_DRIVER::STRING_CMDREQUEST")
  (make-package "NETFT_RDT_DRIVER::STRING_CMDREQUEST"))
(unless (find-package "NETFT_RDT_DRIVER::STRING_CMDRESPONSE")
  (make-package "NETFT_RDT_DRIVER::STRING_CMDRESPONSE"))

(in-package "ROS")





(defclass netft_rdt_driver::String_cmdRequest
  :super ros::object
  :slots (_cmd ))

(defmethod netft_rdt_driver::String_cmdRequest
  (:init
   (&key
    ((:cmd __cmd) "")
    )
   (send-super :init)
   (setq _cmd (string __cmd))
   self)
  (:cmd
   (&optional __cmd)
   (if __cmd (setq _cmd __cmd)) _cmd)
  (:serialization-length
   ()
   (+
    ;; string _cmd
    4 (length _cmd)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _cmd
       (write-long (length _cmd) s) (princ _cmd s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _cmd
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _cmd (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass netft_rdt_driver::String_cmdResponse
  :super ros::object
  :slots (_res ))

(defmethod netft_rdt_driver::String_cmdResponse
  (:init
   (&key
    ((:res __res) "")
    )
   (send-super :init)
   (setq _res (string __res))
   self)
  (:res
   (&optional __res)
   (if __res (setq _res __res)) _res)
  (:serialization-length
   ()
   (+
    ;; string _res
    4 (length _res)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _res
       (write-long (length _res) s) (princ _res s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _res
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _res (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass netft_rdt_driver::String_cmd
  :super ros::object
  :slots ())

(setf (get netft_rdt_driver::String_cmd :md5sum-) "d4463b49bd5bb77dbd8c4356f5dc1c28")
(setf (get netft_rdt_driver::String_cmd :datatype-) "netft_rdt_driver/String_cmd")
(setf (get netft_rdt_driver::String_cmd :request) netft_rdt_driver::String_cmdRequest)
(setf (get netft_rdt_driver::String_cmd :response) netft_rdt_driver::String_cmdResponse)

(defmethod netft_rdt_driver::String_cmdRequest
  (:response () (instance netft_rdt_driver::String_cmdResponse :init)))

(setf (get netft_rdt_driver::String_cmdRequest :md5sum-) "d4463b49bd5bb77dbd8c4356f5dc1c28")
(setf (get netft_rdt_driver::String_cmdRequest :datatype-) "netft_rdt_driver/String_cmdRequest")
(setf (get netft_rdt_driver::String_cmdRequest :definition-)
      "string cmd
---
string res

")

(setf (get netft_rdt_driver::String_cmdResponse :md5sum-) "d4463b49bd5bb77dbd8c4356f5dc1c28")
(setf (get netft_rdt_driver::String_cmdResponse :datatype-) "netft_rdt_driver/String_cmdResponse")
(setf (get netft_rdt_driver::String_cmdResponse :definition-)
      "string cmd
---
string res

")



(provide :netft_rdt_driver/String_cmd "d4463b49bd5bb77dbd8c4356f5dc1c28")


