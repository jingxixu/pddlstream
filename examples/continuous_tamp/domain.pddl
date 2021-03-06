(define (domain pick-and-place)
  (:requirements :strips :equality)
  (:predicates
    (Conf ?q)
    (Block ?b)
    (Pose ?b ?p)
    (Region ?r)
    (Traj ?t)
    (Kin ?b ?q ?p)
    (AtPose ?b ?p)
    (AtConf ?q)
    (Holding ?b)
    (HandEmpty)
    (CFree ?b1 ?p1 ?b2 ?p2)
    (PoseCollision ?b1 ?p1 ?b2 ?p2)
    (TrajCollision ?t ?b2 ?p2)
    (UnsafePose ?b ?p)
    (UnsafeTraj ?t)
    (CanMove)
    (Contained ?b ?p ?r)
    (In ?b ?r)
    (Placeable ?b ?r)
    (Motion ?q1 ?t ?q2)
  )
  (:functions
    (Distance ?q1 ?q2)
  )
  (:action move
    :parameters (?q1 ?t ?q2)
    :precondition (and (Motion ?q1 ?t ?q2)
                       (AtConf ?q1) (CanMove) (not (UnsafeTraj ?t)))
    :effect (and (AtConf ?q2)
                 (not (AtConf ?q1)) (not (CanMove))
             (increase (total-cost) (Distance ?q1 ?q2)))
  )
  (:action pick
    :parameters (?b ?p ?q)
    :precondition (and (Kin ?b ?q ?p)
                       (AtConf ?q) (AtPose ?b ?p) (HandEmpty))
    :effect (and (Holding ?b) (CanMove)
                 (not (AtPose ?b ?p)) (not (HandEmpty))
                 (increase (total-cost) 1))
  )
  (:action place
    :parameters (?b ?p ?q)
    :precondition (and (Kin ?b ?q ?p)
                       (AtConf ?q) (Holding ?b) (not (UnsafePose ?b ?p)))
    :effect (and (AtPose ?b ?p) (HandEmpty) (CanMove)
                 (not (Holding ?b))
                 (increase (total-cost) 1))
  )
  (:derived (UnsafePose ?b1 ?p1)
    (exists (?b2 ?p2) (and (Pose ?b1 ?p1) (Pose ?b2 ?p2) (PoseCollision ?b1 ?p1 ?b2 ?p2)
                            (AtPose ?b2 ?p2)))
  )
  (:derived (UnsafeTraj ?t)
    (exists (?b2 ?p2) (and (Traj ?t) (TrajCollision ?t ?b2 ?p2)
                            (AtPose ?b2 ?p2)))
  )
  (:derived (In ?b ?r)
    (exists (?p) (and (Pose ?b ?p) (Region ?r) (Contained ?b ?p ?r)
                            (AtPose ?b ?p)))
  )
)