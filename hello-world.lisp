
;;;
;;; Load the Bullet engine dynamic libraries
;;;
(progn
  (core:dlopen "/usr/local/lib/libBulletDynamics.dylib")
  (core:dlopen "/usr/local/lib/libBulletCollision.dylib")
  (core:dlopen "/usr/local/lib/libLinearMath.dylib"))

;;;
;;; Load the code that binds Bullet engine classes/methods/functions to claspCL
;;;
(load (merge-pathnames #P"exposeBullet.bc" *load-truename*))

;;;
;;; Setup and step through the simulation of a sphere dropping
;;; on a plane
;;;
(defun sphere-drop-simulation (&optional (steps 300) suppress-output)
  (let* ((broadphase (core:make-cxx-object 'bt:bt-dbvt-broadphase))
	 (collision-configuration (core:make-cxx-object 'bt:bt-default-collision-configuration))
	 (dispatcher (bt:make-bt-collision-dispatcher collision-configuration))
	 (solver (core:make-cxx-object 'bt:bt-sequential-impulse-constraint-solver))
	 (dynamics-world (bt:make-bt-discrete-dynamics-world
			  dispatcher broadphase solver collision-configuration)))
    (bt:set-gravity dynamics-world (bt:make-bt-vector3 0 -10 0))
    (let* ((ground-shape (bt:make-bt-static-plane-shape (bt:make-bt-vector3 0 1 0) 1))
	   (fall-shape (bt:make-bt-sphere-shape 1))
	   (ground-motion-state (bt:make-bt-default-motion-state
				 (bt:make-bt-transform-quaternion
				  (bt:make-bt-quaternion 0 0 0 1)
				  (bt:make-bt-vector3 0 -1 0))))
	   (ground-rigid-body-ci (bt:make-bt-rigid-body/bt-rigid-body-construction-info
				  0 ground-motion-state ground-shape (bt:make-bt-vector3 0 0 0)))
	   (ground-rigid-body (bt:make-bt-rigid-body.bt-rigid-body-construction-info ground-rigid-body-ci)))
      (bt:add-rigid-body dynamics-world ground-rigid-body)
      (let* ((fall-motion-state (bt:make-bt-default-motion-state (bt:make-bt-transform-quaternion
								  (bt:make-bt-quaternion 0 0 0 1)
								  (bt:make-bt-vector3 0 50 0))))
	     (mass 1)
	     (fall-inertia (bt:make-bt-vector3 0 0 0)))
	(bt:calculate-local-inertia fall-shape mass fall-inertia)
	(let* ((fall-rigid-body-ci (bt:make-bt-rigid-body/bt-rigid-body-construction-info
				    mass fall-motion-state fall-shape fall-inertia))
	       (fall-rigid-body (bt:make-bt-rigid-body.bt-rigid-body-construction-info fall-rigid-body-ci))
	       (time-step (/ 1.0 60.0)))
	  (bt:add-rigid-body dynamics-world fall-rigid-body)
	  (loop for i from 0 below steps
	     with transform = (core:make-cxx-object 'bt:bt-transform)
	     do (bt:step-simulation dynamics-world time-step 10 time-step)
	     do (bt:get-world-transform (bt:get-motion-state fall-rigid-body) transform)
	     do (unless suppress-output (format t "sphere height: ~a~%" (bt:get-y (bt:get-origin transform)))))
	  (bt:remove-rigid-body dynamics-world fall-rigid-body)
	  (bt:remove-rigid-body dynamics-world ground-rigid-body))))))

;;;
;;; Run the simulation
;;;
(sphere-drop-simulation)

(format t "Running the simulation using C++~%")
(time (bt:cxx-sphere-drop-simulation 1000000 t))

(format t "Running the simulation using ClaspCL~%")
(time (sphere-drop-simulation 1000000 t))

(core:quit)
