
(cl:in-package :asdf)

(defsystem "scene_analyzer-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "stamped_string" :depends-on ("_package_stamped_string"))
    (:file "_package_stamped_string" :depends-on ("_package"))
  ))