(define (domain pick-and-place)
  (:requirements :strips :equality)

  ; there is no type requirements on the predicate definition
  (:predicates
    ; Static Predicates

    (Conf ?conf)
    (Block ?block)
    (Pose ?block ?pose)
    (Region ?region)
    (Traj ?traj)
    (Kin ?block ?conf ?pose)
    (Placeable ?block ?region)
    (Motion ?conf ?traj ?conf2)

    ; Fluent Predicates

    (AtPose ?block ?pose)
    (AtConf ?conf)
    (Holding ?block)
    (Contained ?block ?pose ?region)
    (HandEmpty)
    (CanMove)   ;prevent double move


    ; Derived predicate

    (UnsafePose ?block ?pose)
    (In ?block ?region)

    ; External predicate, defined in stream

    (PoseCollision ?block ?pose ?block2 ?pose2) ;we only care pose collision while ignoring trajectory collision
  )

  ;External functions, defined in stream

  (:functions
    (Distance ?conf ?conf2)
  )

  (:action move
    :parameters (?conf ?traj ?conf2)
    :precondition (and (Motion ?conf ?traj ?conf2)
                       (AtConf ?conf) (CanMove))
    :effect (and (AtConf ?conf2)
                (not (AtConf ?conf)) (not (CanMove))
                (increase (total-cost) (Distance ?conf ?conf2)))
  )
  (:action pick
    :parameters (?block ?pose ?conf)
    :precondition (and (Kin ?block ?conf ?pose)
                       (AtConf ?conf) (AtPose ?block ?pose) (HandEmpty))
    :effect (and (Holding ?block) (CanMove)
                 (not (AtPose ?block ?pose)) (not (HandEmpty))
                 (increase (total-cost) 1))
  )
  (:action place
    :parameters (?block ?pose ?conf)
    :precondition (and (Kin ?block ?conf ?pose)
                       (AtConf ?conf) (Holding ?block) (not (UnsafePose ?block ?pose)))
    :effect (and (AtPose ?block ?pose) (HandEmpty) (CanMove)
                 (not (Holding ?block))
                 (increase (total-cost) 1))
  )

  (:derived (UnsafePose ?block ?pose)
    (exists (?block2 ?pose2) (and (Pose ?block ?pose) (Pose ?block2 ?pose2)
                                  (PoseCollision ?block ?pose ?block2 ?pose2)
                                  (AtPose ?block2 ?pose2)))
  )
  (:derived (In ?block ?region)
    (exists (?pose) (and (Pose ?block ?pose) (Region ?region) (Contained ?block ?pose ?region)
                            (AtPose ?block ?pose)))
  )
)