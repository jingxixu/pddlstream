(define (stream pick-and-place)

  ; External function
  (:function (Distance ?conf ?conf2)
    (and (Conf ?conf) (Conf ?conf2))
  )

  ; External Predicate
  (:predicate (PoseCollision ?block ?pose ?block2 ?pose2)
    (and (Pose ?block ?pose) (Pose ?block2 ?pose2))
  )

  ;Streams
  (:stream sample-pose
    :inputs (?block ?region)
    :domain (Placeable ?block ?region)
    :outputs (?pose)
    :certified (and (Pose ?block ?pose) (Contained ?block ?pose ?region))
  )
  (:stream inverse-kinematics
    :inputs (?block ?pose)
    :domain (Pose ?block ?pose)
    :outputs (?conf)
    :certified (and (Conf ?conf) (Kin ?block ?conf ?pose))
  )
  (:stream plan-motion
    :inputs (?conf ?conf2)
    :domain (and (Conf ?conf) (Conf ?conf2))
    :outputs (?traj)
    :certified (and (Traj ?traj) (Motion ?conf ?traj ?conf2))
  )
)