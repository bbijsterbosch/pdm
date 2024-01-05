from mpscenes.goals.static_sub_goal import StaticSubGoal
from mpscenes.goals.dynamic_sub_goal import DynamicSubGoal
from mpscenes.goals.goal_composition import GoalComposition

goal1Dict = {
    "weight": 1.0,
    "is_primary_goal": True,
    "indices": [0, 1, 2],
    "parent_link": 0,
    "child_link": 3,
    "desired_position": [6.0, -3.0, 0.0],
    "epsilon": 0.2,
    "type": "staticSubGoal",
    'high': [3, 3, 0.1],
    'low': [-3, -3, 0.1],
}

goal_pos = goal1Dict["desired_position"]
goal1 = StaticSubGoal(name="goal1", content_dict=goal1Dict)
