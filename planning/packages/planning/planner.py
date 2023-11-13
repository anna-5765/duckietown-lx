from typing import List
import math

from aido_schemas import Context, FriendlyPose
from dt_protocols import (
    PlacedPrimitive,
    PlanningQuery,
    PlanningResult,
    PlanningSetup,
    PlanStep,
    Rectangle,
)

__all__ = ["Planner"]

def connect_poses (ps: PlanningSetup, a: FriendlyPose, b: FriendlyPose):

        # a plan is a list of plan steps
        plan: List[PlanStep] = []
        
        # find turn angle and distance difference (a is start b is goal)
        dist_x = b.x - a.x                                          # x distance from start to goal
        dist_y = b.y - a.y                                          # y distance from start to goal
        theta_goal_turn = math.degrees(math.atan2(dist_y, dist_x))  # angle to goal position (in degrees)
        distance_start_goal = math.sqrt(dist_x**2+dist_y**2)        # distance from start pos to goal pos
        theta_goal_orient = b.theta_deg - theta_goal_turn           # difference in move angle to goal angle
    
        # write steps - for empty environment, turn toward goal position, move forward to x,y coords, orient at goal
        goal_turn = PlanStep(
            duration = abs(theta_goal_turn/ps.max_angular_velocity_deg_s),
            velocity_x_m_s = 0.0,
            angular_velocity_deg_s = ps.max_angular_velocity_deg_s if theta_goal_turn > 0 else -ps.max_angular_velocity_deg_s
        )
        goal_move = PlanStep(
            duration = distance_start_goal/ps.max_linear_velocity_m_s,
            velocity_x_m_s = ps.max_linear_velocity_m_s,
            angular_velocity_deg_s = 0.0
        )
        goal_orient = PlanStep(
            duration = abs(theta_goal_orient/ps.max_angular_velocity_deg_s),
            velocity_x_m_s = 0.0,
            angular_velocity_deg_s = ps.max_angular_velocity_deg_s if theta_goal_orient > 0 else -ps.max_angular_velocity_deg_s
        )

        # make plan
        plan.append(goal_turn)
        plan.append(goal_move)
        plan.append(goal_orient)

        return plan

class Planner:
    params: PlanningSetup

    def init(self, context: Context):
        context.info("init()")

    def on_received_set_params(self, context: Context, data: PlanningSetup):
        context.info("initialized")
        self.params = data

        # This is the interval of allowed linear velocity
        # Note that min_velocity_x_m_s and max_velocity_x_m_s might be different.
        # Note that min_velocity_x_m_s may be 0 in advanced exercises (cannot go backward)
        max_velocity_x_m_s: float = self.params.max_linear_velocity_m_s
        min_velocity_x_m_s: float = self.params.min_linear_velocity_m_s

        # This is the max curvature. In earlier exercises, this is +inf: you can turn in place.
        # In advanced exercises, this is less than infinity: you cannot turn in place.
        max_curvature: float = self.params.max_curvature

        # these have the same meaning as the collision exercises
        body: List[PlacedPrimitive] = self.params.body
        environment: List[PlacedPrimitive] = self.params.environment

        # these are the final tolerances - the precision at which you need to arrive at the goal
        tolerance_theta_deg: float = self.params.tolerance_theta_deg
        tolerance_xy_m: float = self.params.tolerance_xy_m

        # For convenience, this is the rectangle that contains all the available environment,
        # so you don't need to compute it
        bounds: Rectangle = self.params.bounds

    def on_received_query(self, context: Context, data: PlanningQuery):
        # A planning query is a pair of initial and goal poses
        start: FriendlyPose = data.start
        goal: FriendlyPose = data.target

        # You start at the start pose. You must reach the goal with a tolerance given by
        # tolerance_xy_m and tolerance_theta_deg.

        # You need to declare if it is feasible or not
        feasible = True

        if not feasible:
            # If it's not feasible, just return this.
            result: PlanningResult = PlanningResult(False, None)
            context.write("response", result)
            return

        # If it is feasible you need to provide a plan.
        plan = connect_poses(self.params, start, goal)

        result: PlanningResult = PlanningResult(feasible, plan)
        context.write("response", result)
