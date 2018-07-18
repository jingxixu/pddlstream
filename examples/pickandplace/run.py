import numpy as np
from pddlstream.algorithms.focused import solve_focused
from pddlstream.algorithms.incremental import solve_incremental
from pddlstream.language.conversion import And, Equal
from pddlstream.language.generator import from_gen_fn, from_fn
from pddlstream.utils import print_solution, read, INF, get_file_path


def main():
    domain_pddl = read(get_file_path(__file__, 'domain.pddl'))
    stream_pddl = read(get_file_path(__file__, 'stream.pddl'))

    # starting objects
    start_conf = np.array([0, 5])
    p0 = np.array([0, 0])
    goal_region = np.array([5, 10])
    b0 = 'block0'
    ground = np.array([-5, 15])

    initial_atoms = [
        ('CanMove',),
        ('Conf', start_conf),
        ('AtConf', start_conf),
        ('HandEmpty',),
        ('Region', goal_region),
        ('Block', b0),
        ('Pose', b0, p0),
        ('AtPose', b0, p0),
        ('Region', ground),
        ('Placeable', b0, ground),
        ('Placeable', b0, goal_region)]

    goal_literals = [
        ('HandEmpty', ),
        ('In', b0, goal_region),
        ('AtConf', start_conf)
    ]

    goal_formula = And(*goal_literals)

    ## environments
    # The pose of block is the center of bottom line
    # The pose of the gripper is the center of mass
    BLOCK_WIDTH = 2
    BLOCK_HEIGHT = BLOCK_WIDTH
    GRIPPER_WIDTH = 2
    GRIPPER_HEIGHT = GRIPPER_WIDTH
    GRASP = -np.array([0, BLOCK_HEIGHT + GRIPPER_HEIGHT / 2])

    def distance(q1, q2):
        return int(np.linalg.norm(q1-q2))

    def posecollision(blk1, pose1, blk2, pose2):
        i1 = pose1[0] * np.ones(2) + np.array([-BLOCK_WIDTH, +BLOCK_WIDTH]) / 2.
        i2 = pose2[0] * np.ones(2) + np.array([-BLOCK_WIDTH, +BLOCK_WIDTH]) / 2.
        return (i2[0] <= i1[1]) and (i1[0] <= i2[1])

    def sample_pose(blk, region):
        return ((np.random.uniform(region[0]+BLOCK_WIDTH/2, region[1]-BLOCK_WIDTH/2), 0), )

    def inverse_kinematics(blk, pose):
        return (pose - GRASP, )

    def plan_motion(conf1, conf2):
        traj = [conf1, conf2]
        return (traj, )


    constant_map = {}
    # Upper case does not matter at all
    stream_map = {'distance': distance,
                  'posecollision': posecollision,
                  'sample-pose': from_fn(sample_pose),
                  'inverse-kinematics': from_fn(inverse_kinematics),
                  'plan-motion': from_fn(plan_motion)}

    # A pddlstream problem
    problem = domain_pddl, constant_map, stream_pddl, stream_map, initial_atoms, goal_formula
    # solution = solve_incremental(problem, unit_costs=False)
    solution = solve_incremental(problem, unit_costs=False)
    # plan, cost, evaluations = solution
    print_solution(solution)

if __name__ == '__main__':
    main()