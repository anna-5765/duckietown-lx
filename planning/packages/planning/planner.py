from typing import List
import math
import networkx as nx
import itertools
import geometry as geo

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

def connect_poses (G, ps: PlanningSetup, a: FriendlyPose, b: FriendlyPose):
    # a plan is a list of plan steps
    plan: List[PlanStep] = []

    # ## Empty Environment (has 4 placed primitives in environment list)
    # if len(ps.environment) == 4:
    #     # find turn angle and distance difference (a is start b is goal)
    #     dist_x = b.x - a.x                                          # x distance from start to goal
    #     dist_y = b.y - a.y                                          # y distance from start to goal
    #     theta_goal_turn = math.degrees(math.atan2(dist_y, dist_x))  # angle to goal position (in degrees)
    #     distance_start_goal = math.sqrt(dist_x**2+dist_y**2)        # distance from start pos to goal pos
    #     theta_goal_orient = b.theta_deg - theta_goal_turn           # difference in move angle to goal angle
                
    #     # write steps - for empty environment, turn toward goal position, move forward to x,y coords, orient at goal
    #     goal_turn = PlanStep(
    #         duration = abs(theta_goal_turn/ps.max_angular_velocity_deg_s),
    #         velocity_x_m_s = 0.0,
    #         angular_velocity_deg_s = ps.max_angular_velocity_deg_s if theta_goal_turn > 0 else -ps.max_angular_velocity_deg_s
    #     )
    #     goal_move = PlanStep(
    #         duration = distance_start_goal/ps.max_linear_velocity_m_s,
    #         velocity_x_m_s = ps.max_linear_velocity_m_s,
    #         angular_velocity_deg_s = 0.0
    #     )
    #     goal_orient = PlanStep(
    #         duration = abs(theta_goal_orient/ps.max_angular_velocity_deg_s),
    #         velocity_x_m_s = 0.0,
    #         angular_velocity_deg_s = ps.max_angular_velocity_deg_s if theta_goal_orient > 0 else -ps.max_angular_velocity_deg_s
    #     )

    #     # make plan
    #     plan.append(goal_turn)
    #     plan.append(goal_move)
    #     plan.append(goal_orient)

    ## Static Obstacles
    if len(ps.environment) > 4: # change to > 4 when troubleshooting over for non-empty environment
        start_node = closest_node(G, a)
        goal_node = closest_node(G, b)
        node_path = nx.dijkstra_path(G, start_node, goal_node) # find path of nodes from start to goal
        pose_data = [G.nodes[node]['q'][:3,2] for node in node_path] # convert sequence of nodes to poses
        
        # def se2_to_friendly_pose(q):
        #     x,y = q[:2, 2]
        #     theta_deg = math.degrees(math.atan2(q[2,2], q[1,2]))
        #     return FriendlyPose(x, y, theta_deg)
        # pose_data = [se2_to_friendly_pose(G.nodes[node]['q']) for node in node_path]
    

        for node in range(1, len(pose_data)):
            dist_x = pose_data[node][0] - pose_data[node-1][0]                                         
            dist_y = pose_data[node][1] - pose_data[node-1][1]                                         
            theta_goal_turn = math.degrees(math.atan2(dist_y, dist_x))  
            distance_start_goal = math.sqrt(dist_x**2+dist_y**2)
            pose_data[node][2] = theta_goal_turn        
                
            if node == 1:
                step_turn = PlanStep(
                    duration = abs((theta_goal_turn - a.theta_deg)/ps.max_angular_velocity_deg_s),
                    velocity_x_m_s = 0.0,
                    angular_velocity_deg_s = ps.max_angular_velocity_deg_s if (theta_goal_turn - a.theta_deg) > 0 else -ps.max_angular_velocity_deg_s
                )
                step_move = PlanStep(
                    duration = distance_start_goal/ps.max_linear_velocity_m_s,
                    velocity_x_m_s = ps.max_linear_velocity_m_s,
                    angular_velocity_deg_s = 0.0
                )
                # add to plan
                plan.append(step_turn)
                plan.append(step_move)
            
            elif node > 1:
                if not pose_data[node][2] == pose_data[node-1][2]:
                    step_turn = PlanStep(
                        duration = abs((theta_goal_turn - pose_data[node-1][2])/ps.max_angular_velocity_deg_s),
                        velocity_x_m_s = 0.0,
                        angular_velocity_deg_s = ps.max_angular_velocity_deg_s if (theta_goal_turn - pose_data[node-1][2]) > 0 else -ps.max_angular_velocity_deg_s
                    )
                    plan.append(step_turn)
                
                step_move = PlanStep(
                    duration = distance_start_goal/ps.max_linear_velocity_m_s,
                    velocity_x_m_s = ps.max_linear_velocity_m_s,
                    angular_velocity_deg_s = 0.0
                )
                plan.append(step_move)

                if node == len(pose_data): # last node, orient at goal position
                    theta_goal_orient = b.theta_deg - pose_data[node-1][2]  # difference in move angle to goal angle

                    goal_orient = PlanStep(
                        duration = abs(theta_goal_orient/ps.max_angular_velocity_deg_s),
                        velocity_x_m_s = 0.0,
                        angular_velocity_deg_s = ps.max_angular_velocity_deg_s if theta_goal_orient > 0 else -ps.max_angular_velocity_deg_s
                    )

                    # add to plan
                    plan.append(goal_orient)
    
    # print('NODE PATH', node_path)
    # print('POSE DATA', pose_data)
    # print('START', a)
    # print('GOAL', b)
    # print('ENVIRONMENT', ps.environment)

    return plan

def closest_node(G, target_pos):
    min_distance = float('inf')
    closest = None

    for node in G.nodes.data():
        node_name, node_data = node
        node_pos = node_data['q'][:2, 2]
        dist = math.dist([node_pos[0], node_pos[1]], [target_pos.x, target_pos.y])
        if dist < min_distance:
            min_distance = dist
            closest = node_name

    return closest

def collision_check(node, env: List[PlacedPrimitive]):
    ## Empty Environment
    if not env:
        collision = False
    ## Obstacles Exist
    else:
        for primitive in env:
            if isinstance(primitive.primitive, Rectangle):
                if (
                    node[0] <= primitive.pose.x + primitive.primitive.xmax
                    and node[0] >= primitive.pose.x + primitive.primitive.xmin
                    and node[1] <= primitive.pose.y + primitive.primitive.ymax 
                    and node[1] >= primitive.pose.y + primitive.primitive.ymin
                ):
                    collision = True
                else: 
                    collision = False
            else: # Circle
                distance = math.dist(node, [primitive.pose.x, primitive.pose.y])
                if distance < primitive.primitive.radius:
                    collision = True
                else: 
                    collision = False
    return bool(collision)

def pose_network(ps: PlanningSetup):
    # Base grid size off of acceptable tolerance from goal
    grid_size = ps.tolerance_xy_m
    # Find height and width of environment bounds
    H, W = round((abs(ps.bounds.ymin)+ps.bounds.ymax)/grid_size), round((abs(ps.bounds.xmin)+ps.bounds.xmax)/grid_size)

    # Create grid pose network
    G = nx.MultiDiGraph()
    for i, j in itertools.product(range(H), range(W)):
        node_name = (i,j)
        # create pose
        q = geo.SE2_from_translation_angle((i*grid_size, j*grid_size), 0)
        G.add_node(node_name, q=q)
    
    # Create nework connections
    for i,j in itertools.product(range(H), range(W)):
        # find neighbor nodes
        for d in [(+1,0), (0,+1), (+1,+1), (-1, +1)]:
            i2, j2 = i+d[0], j+d[1]
            # if neighbor exists add the connection
            if (i2, j2) in G:
                # pose of first node
                q1 = G.nodes[(i,j)]['q']
                # pose of second node
                q2 = G.nodes[(i2,j2)]['q']
                # relative pose
                relative_pose = geo.SE2.multiply(geo.SE2.inverse(q1), q2)
                # label
                label = geo.SE2.friendly(relative_pose)
                # add the edge with two properties "label" and "relative pose"
                G.add_edge((i,j), (i2,j2), label=label, relative_pose=relative_pose)
                # add inverse edges
                rinv = geo.SE2.inverse(relative_pose)
                linv = geo.SE2.friendly(rinv)
                G.add_edge((i2, j2), (i,j), label=linv, relative_pose=rinv)
    return G

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

        # Create pose network
        G = pose_network(self.params)

        # Remove nodes if collision
        for node in list(G.nodes()):
            collision = collision_check(node, self.params.environment)
            if collision:
                G.remove_node(node)

        # You need to declare if it is feasible or not
        start_node = closest_node(G, start)
        goal_node = closest_node(G, goal)
        if nx.has_path(G, start_node, goal_node):
            feasible = True
        else:
            feasible = False

        # print('has path', feasible)

        if not feasible:
            # If it's not feasible, just return this.
            result: PlanningResult = PlanningResult(False, None)
            context.write("response", result)
            return

        # If it is feasible you need to provide a plan.
        plan = connect_poses(G, self.params, start, goal)
        # print('!!! THIS IS THE PLAN !!!', plan)

        result: PlanningResult = PlanningResult(feasible, plan)
        context.write("response", result)
