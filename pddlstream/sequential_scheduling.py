from pddlstream.conversion import evaluation_from_fact, obj_from_pddl, obj_from_pddl_plan
from pddlstream.fast_downward import task_from_domain_problem, get_problem, solve_from_task, get_init
from pddlstream.simultaneous_scheduling import get_stream_actions
from pddlstream.utils import Verbose, find


def sequential_stream_plan(evaluations, goal_expression, domain, stream_results, **kwargs):
    opt_evaluations = set(evaluations)
    for stream_result in stream_results:
        for fact in stream_result.get_certified():
            opt_evaluations.add(evaluation_from_fact(fact))
    task = task_from_domain_problem(domain, get_problem(opt_evaluations, goal_expression, domain))
    action_plan, _ = solve_from_task(task, **kwargs)
    if action_plan is None:
        return None, None
    real_init = get_init(evaluations)
    opt_facts = set(task.init) - set(real_init)

    import pddl
    import pddl_to_prolog
    import build_model
    import instantiate
    with Verbose(False):
        model = build_model.compute_model(pddl_to_prolog.translate(task))
        fluent_facts = instantiate.get_fluent_facts(task, model) | opt_facts
    task.init = real_init
    init_facts = set(task.init)
    type_to_objects = instantiate.get_objects_by_type(task.objects, task.types)
    task.actions, stream_result_from_name = get_stream_actions(stream_results)

    # TODO: add ordering constraints to simplify the optimization
    action_from_name = {}
    for i, (name, args) in enumerate(action_plan):
        action = find(lambda a: a.name == name, domain.actions)
        assert(len(action.parameters) == len(args))
        #parameters = action.parameters[:action.num_external_parameters]
        var_mapping = {p.name: a for p, a in zip(action.parameters, args)}
        new_name = '{}-{}'.format(name, i)
        new_parameters = action.parameters[len(args):]
        new_preconditions = []
        action.precondition.instantiate(var_mapping, init_facts, fluent_facts, new_preconditions)
        new_effects = []
        for eff in action.effects:
            eff.instantiate(var_mapping, init_facts, fluent_facts, type_to_objects, new_effects)
        new_effects = [pddl.Effect([], pddl.Conjunction(conditions), effect)
                      for conditions, effect in new_effects]
        task.actions.append(pddl.Action(new_name, new_parameters, 0,
                                   pddl.Conjunction(new_preconditions), new_effects, 1))
        action_from_name[new_name] = (name, map(obj_from_pddl, args))

    combined_plan, _ = solve_from_task(task, **kwargs)
    if combined_plan is None:
        return None, obj_from_pddl_plan(action_plan)
    stream_plan = []
    action_plan = []
    for name, args in combined_plan:
        if name in stream_result_from_name:
            stream_plan.append(stream_result_from_name[name])
        else:
            action_plan.append(action_from_name[name])
    return stream_plan, action_plan