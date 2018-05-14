from __future__ import print_function

import os
from collections import namedtuple

import numpy as np
import cProfile
import pstats

from pddlstream.conversion import And
from pddlstream.incremental import solve_incremental
from pddlstream.stream import from_fn, from_test, StreamInfo, from_gen_fn
from pddlstream.utils import print_solution, user_input, read, INF

from examples.discrete_belief.dist import DDist, MixtureDD, DeltaDist, UniformDist

BeliefProblem = namedtuple('BeliefProblem', ['initial', 'goal', 'locations',
                                             'p_move_s', 'p_look_fp', 'p_look_fn'])

def get_belief_problem(num_locs, deterministic, observable):
    if observable:
        initial = [
          ('o1', 'l1', 1.0),
          ('o2', 'l2', 1.0),
        ]
    else:
        initial = [
          ('o1', 'l1', 0.6),
          ('o2', 'l2', 0.9),
        ]

    #goal = [('o1', 'l1', 0.95)]
    goal = [('o1', 'l0', 0.95)]
    #goal = [('o1', 'l2', 0.95)]

    locations = {'l0'}

    if deterministic:
        p_move_s = 1.0
        p_look_fp = p_look_fn = 0.0
    else:
        p_move_s = 0.8
        p_look_fp = p_look_fn = 0.1

    return BeliefProblem(initial, goal, locations,
                         p_move_s, p_look_fn, p_look_fp)

##################################################

def ge_fn(d, l, p):
    return bool(d.prob(l) >= p) # TODO: numpy boolean

def get_move_fn(belief_problem):
    def fn(control_loc1, control_loc2, d1):
        d2 = d1.copy()
        def transition_fn(loc1):
            # P(s2=loc2 | s1=loc1, a=(control_loc1, control_loc2))
            perfect_d = DeltaDist(control_loc2)
            fail_d = DeltaDist(loc1)
            #fail_d = UniformDist(locations)
            #return MixtureDD(perfect_d, fail_d, belief_problem.p_move_s)
            if control_loc1 == loc1:
                return MixtureDD(perfect_d, fail_d, belief_problem.p_move_s)
            return fail_d
        d2.transitionUpdate(transition_fn)
        return (d2,)
    return fn

def get_look_fn(belief_problem):
    def fn(control_loc, d1):
        if d1.prob(control_loc) == 0:  # Not possible to generate observation
            return None
        def observation_fn(loc1):
            # P(o=hit | s1=loc1, a=control_loc)
            if loc1 == control_loc:
                return DDist({True: 1 - belief_problem.p_look_fn,
                              False: belief_problem.p_look_fn})
            return DDist({True: belief_problem.p_look_fp,
                          False: 1 - belief_problem.p_look_fp})
        d2 = d1.copy()
        obs = True
        d2.obsUpdate(observation_fn, obs)
        return (d2,)
    return fn

def to_pddlstream(belief_problem):
    objects = {o for (o, _, _) in belief_problem.initial}
    locations = {l for (_, l, _) in belief_problem.initial + belief_problem.goal} | \
                set(belief_problem.locations)
    uniform = UniformDist(locations)
    initial_bel = {o: MixtureDD(DeltaDist(l), uniform, p) for o, l, p in belief_problem.initial}

    # TODO: separate pick and place for move
    init = \
        [('Obj', o) for o in objects] + \
        [('Location', l) for l in locations]
    for o, d in initial_bel.items():
        init += [('BLoc', o, d), ('Dist', d)]
    for (o, l, p) in belief_problem.goal:
        init += [('Obj', o), ('Location', l), ('Prob', p)]
    goal_literals = [('BLocGE', o, l, p) for (o, l, p) in belief_problem.goal]
    goal = And(*goal_literals)

    directory = os.path.dirname(os.path.abspath(__file__))
    domain_pddl = read(os.path.join(directory, 'domain.pddl'))
    stream_pddl = read(os.path.join(directory, 'stream.pddl'))
    constant_map = {}
    stream_map = {
        'ge': from_test(ge_fn),
        'prob-after-move': from_fn(get_move_fn(belief_problem)),
        'prob-after-look': from_fn(get_look_fn(belief_problem)),
    }

    return domain_pddl, constant_map, stream_pddl, stream_map, init, goal

def main(num_locs=5, deterministic=False, observable=False, focused=True, custom_bound=True):

    belief_problem = get_belief_problem(num_locs, deterministic, observable)
    pddlstream_problem = to_pddlstream(belief_problem)

    solution = solve_incremental(pddlstream_problem)
    print_solution(solution)

if __name__ == '__main__':
    main(deterministic=False, observable=False, focused=True, custom_bound=True)