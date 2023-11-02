import itertools
import random
import numpy as np
import math
from typing import List

from aido_schemas import Context, FriendlyPose
from dt_protocols import (
    Circle,
    CollisionCheckQuery,
    CollisionCheckResult,
    MapDefinition,
    PlacedPrimitive,
    Rectangle,
)

__all__ = ["CollisionChecker"]


class CollisionChecker:
    params: MapDefinition

    def init(self, context: Context):
        context.info("init()")

    def on_received_set_params(self, context: Context, data: MapDefinition):
        context.info("initialized")
        self.params = data

    def on_received_query(self, context: Context, data: CollisionCheckQuery):
        collided = check_collision(
            environment=self.params.environment, robot_body=self.params.body, robot_pose=data.pose
        )
        result = CollisionCheckResult(collided)
        context.write("response", result)


def check_collision(
    environment: List[PlacedPrimitive], robot_body: List[PlacedPrimitive], robot_pose: FriendlyPose
) -> bool:
    # This is just some code to get you started, but you don't have to follow it exactly

    # You can start by rototranslating the robot_body by the robot_pose
    rototranslated_robot: List[PlacedPrimitive] = []
    for placed_primitive in robot_body:
        # Translate
        new_x = placed_primitive.pose.x + robot_pose.x
        new_y = placed_primitive.pose.y + robot_pose.y

        # Rotate
        # TODO: Do I need to rotate the x and y coord too?
        new_theta = placed_primitive.pose.theta_deg + robot_pose.theta_deg

        # Create new placed primitive
        new_pose = FriendlyPose(new_x, new_y, new_theta)
        rototranslated_primitive = PlacedPrimitive(new_pose, placed_primitive.primitive)

        rototranslated_robot.append(rototranslated_primitive)


    # Then, call check_collision_list to see if the robot collides with the environment
    collided = check_collision_list(rototranslated_robot, environment)

    # Return true when argument true, otherwise false
    return bool(collided)


def check_collision_list(
    rototranslated_robot: List[PlacedPrimitive], environment: List[PlacedPrimitive]
) -> bool:
    # This is just some code to get you started, but you don't have to follow it exactly
    for robot, envObject in itertools.product(rototranslated_robot, environment):
        if check_collision_shape(robot, envObject):
            return True

    return False


def check_collision_shape(a: PlacedPrimitive, b: PlacedPrimitive) -> bool:
    # This is just some code to get you started, but you don't have to follow it exactly
    if isinstance(a.primitive, Circle) and isinstance(b.primitive, Circle):
        cir_center_dist = math.sqrt((b.pose.x-a.pose.x)**2+(b.pose.y-a.pose.y)**2)
        if cir_center_dist <= a.primitive.radius + b.primitive.radius:
            shape_collided = True
        else: 
            shape_collided = False
    elif isinstance(a.primitive, Rectangle) and isinstance(b.primitive, Circle):
        # a robot b circle in environment
        # approx a as circle inside rectangle (r=xmax)
        cir_center_dist = math.sqrt((b.pose.x-a.pose.x)**2+(b.pose.y-a.pose.y)**2)
        if cir_center_dist < a.primitive.xmax + b.primitive.radius:
                shape_collided = True
        else: 
            shape_collided = False

        # ## Separating Axis Theorem
        # # Find corners of rectangle a
        # a_theta_rad = np.deg2rad(a.pose.theta_deg)

        # # apply rotation
        # a_r1 = [a.primitive.xmax*math.cos(a_theta_rad)-a.primitive.ymax*math.sin(a_theta_rad), a.primitive.xmax*math.sin(a_theta_rad)+a.primitive.ymax*math.cos(a_theta_rad)]
        # a_r2 = [a.primitive.xmin*math.cos(a_theta_rad)-a.primitive.ymax*math.sin(a_theta_rad), a.primitive.xmin*math.sin(a_theta_rad)+a.primitive.ymax*math.cos(a_theta_rad)]
        # a_r3 = [a.primitive.xmin*math.cos(a_theta_rad)-a.primitive.ymin*math.sin(a_theta_rad), a.primitive.xmin*math.sin(a_theta_rad)+a.primitive.ymin*math.cos(a_theta_rad)]
        # a_r4 = [a.primitive.xmax*math.cos(a_theta_rad)-a.primitive.ymin*math.sin(a_theta_rad), a.primitive.xmax*math.sin(a_theta_rad)+a.primitive.ymin*math.cos(a_theta_rad)]

        # # apply translation 
        # a_c1 = [a_r1[0]+a.pose.x, a_r1[1]+a.pose.y]
        # a_c2 = [a_r2[0]+a.pose.x, a_r2[1]+a.pose.y]
        # a_c3 = [a_r3[0]+a.pose.x, a_r3[1]+a.pose.y]
        # a_c4 = [a_r4[0]+a.pose.x, a_r4[1]+a.pose.y]

        # # Find axes of rectangle
        # axis1_a = np.array([a_c1[0]-a_c2[0], a_c1[1]-a_c2[1]])
        # axis2_a = np.array([a_c1[0]-a_c4[0], a_c1[1]-a_c4[1]])

        # # Project points onto each axis
        # b_center = [b.pose.x, b.pose.y]
        # r_points = [a_c1, a_c2, a_c3, a_c4]
        # axes = [axis1_a, axis2_a]
        # proj_matrix_r = [[], []]
        # proj_matrix_c = [[], []]

        # # rectangle
        # for axis in range(len(axes)):
        #     for point in r_points:
        #         projection = np.dot(point, axes[axis])
        #         proj_matrix_r[axis].append(projection)
        
        # # circle
        # for axis in range(len(axes)):
        #     projection = np.dot(b_center, axes[axis])
        #     min_projection = projection - b.primitive.radius
        #     max_projection = projection + b.primitive.radius
        #     proj_matrix_c[axis].append(min_projection)
        #     proj_matrix_c[axis].append(max_projection)

        # # Use max and min values from projections to determine overlap
        # condition1 = min(proj_matrix_r[0])<= max(proj_matrix_c[0]) and max(proj_matrix_r[0])>= min(proj_matrix_c[0])
        # condition2 = min(proj_matrix_r[1])<= max(proj_matrix_c[1]) and max(proj_matrix_r[1])>= min(proj_matrix_c[1])
        
        
        # print("radius", b.primitive.radius)
        # print("proj matrix circle", proj_matrix_c)
        # print("proj matrix rectangle", proj_matrix_r)


        # if condition1 and condition2:
        #     shape_collided = True
        # else: 
        #     shape_collided = False   

    elif isinstance(a.primitive, Rectangle) and isinstance(b.primitive, Rectangle):
        # # first approx all shapes as circle inside rectangle (using r=xmax)
        # cir_center_dist = math.sqrt((b.pose.x-a.pose.x)**2+(b.pose.y-a.pose.y)**2)
        # if cir_center_dist <= a.primitive.xmax + b.primitive.xmax:
        #     shape_collided = True

        ## Separating Axis Theorem
        # Find corners of rectangles
        # a
        a_theta_rad = np.deg2rad(a.pose.theta_deg)

        # apply rotation
        a_r1 = [a.primitive.xmax*math.cos(a_theta_rad)-a.primitive.ymax*math.sin(a_theta_rad), a.primitive.xmax*math.sin(a_theta_rad)+a.primitive.ymax*math.cos(a_theta_rad)]
        a_r2 = [a.primitive.xmin*math.cos(a_theta_rad)-a.primitive.ymax*math.sin(a_theta_rad), a.primitive.xmin*math.sin(a_theta_rad)+a.primitive.ymax*math.cos(a_theta_rad)]
        a_r3 = [a.primitive.xmin*math.cos(a_theta_rad)-a.primitive.ymin*math.sin(a_theta_rad), a.primitive.xmin*math.sin(a_theta_rad)+a.primitive.ymin*math.cos(a_theta_rad)]
        a_r4 = [a.primitive.xmax*math.cos(a_theta_rad)-a.primitive.ymin*math.sin(a_theta_rad), a.primitive.xmax*math.sin(a_theta_rad)+a.primitive.ymin*math.cos(a_theta_rad)]

        # apply translation 
        a_c1 = [a_r1[0]+a.pose.x, a_r1[1]+a.pose.y]
        a_c2 = [a_r2[0]+a.pose.x, a_r2[1]+a.pose.y]
        a_c3 = [a_r3[0]+a.pose.x, a_r3[1]+a.pose.y]
        a_c4 = [a_r4[0]+a.pose.x, a_r4[1]+a.pose.y]
        
        # b
        b_theta_rad = np.deg2rad(b.pose.theta_deg)
        
        # apply rotation
        b_r1 = [b.primitive.xmax*math.cos(b_theta_rad)-b.primitive.ymax*math.sin(b_theta_rad), b.primitive.xmax*math.sin(b_theta_rad)+b.primitive.ymax*math.cos(b_theta_rad)]
        b_r2 = [b.primitive.xmin*math.cos(b_theta_rad)-b.primitive.ymax*math.sin(b_theta_rad), b.primitive.xmin*math.sin(b_theta_rad)+b.primitive.ymax*math.cos(b_theta_rad)]
        b_r3 = [b.primitive.xmin*math.cos(b_theta_rad)-b.primitive.ymin*math.sin(b_theta_rad), b.primitive.xmin*math.sin(b_theta_rad)+b.primitive.ymin*math.cos(b_theta_rad)]
        b_r4 = [b.primitive.xmax*math.cos(b_theta_rad)-b.primitive.ymin*math.sin(b_theta_rad), b.primitive.xmax*math.sin(b_theta_rad)+b.primitive.ymin*math.cos(b_theta_rad)]

        # apply translation 
        b_c1 = [b_r1[0]+b.pose.x, b_r1[1]+b.pose.y]
        b_c2 = [b_r2[0]+b.pose.x, b_r2[1]+b.pose.y]
        b_c3 = [b_r3[0]+b.pose.x, b_r3[1]+b.pose.y]
        b_c4 = [b_r4[0]+b.pose.x, b_r4[1]+b.pose.y]

        # Find axes of rectangle
        axis1_a = np.array([a_c1[0]-a_c2[0], a_c1[1]-a_c2[1]])
        axis2_a = np.array([a_c1[0]-a_c4[0], a_c1[1]-a_c4[1]]) 
        axis3_b = np.array([b_c2[0]-b_c3[0], b_c2[1]-b_c3[1]])
        axis4_b = np.array([b_c1[0]-b_c2[0], b_c1[1]-b_c2[1]])

        # Project 4 corners on each axis
        proj_matrix = [[], [], [], []]
        corners = [a_c1, a_c2, a_c3, a_c4, b_c1, b_c2, b_c3, b_c4]
        axes = [axis1_a, axis2_a, axis3_b, axis4_b]
        # print('pose a', a.pose.x, a.pose.y, a.pose.theta_deg, a.primitive.xmax, a.primitive.xmin, a.primitive.ymax, a.primitive.ymin)
        # print('pose b', b.pose.x, b.pose.y, b.pose.theta_deg, b.primitive.xmax, b.primitive.xmin, b.primitive.ymax, b.primitive.ymin)
        # print('corners', corners)
        # print('axes', axes)

        for axis in range(len(axes)):
            for corner in corners:
                projection_scalar = np.dot(corner, axes[axis])
                proj_matrix[axis].append(projection_scalar)

        # Use max and min values from projections to determine overlap
        # print('Proj_Matrix', proj_matrix)
        # print(min(proj_matrix[0][4:]), max(proj_matrix[0][0:4]), max(proj_matrix[0][4:]), min(proj_matrix[0][0:4]))
        condition1 = min(proj_matrix[0][4:])<= max(proj_matrix[0][0:4]) and max(proj_matrix[0][4:])>= min(proj_matrix[0][0:4])
        condition2 = min(proj_matrix[1][4:])<= max(proj_matrix[1][0:4]) and max(proj_matrix[1][4:])>= min(proj_matrix[1][0:4])
        condition3 = min(proj_matrix[2][4:])<= max(proj_matrix[2][0:4]) and max(proj_matrix[2][4:])>= min(proj_matrix[2][0:4])
        condition4 = min(proj_matrix[3][4:])<= max(proj_matrix[3][0:4]) and max(proj_matrix[3][4:])>= min(proj_matrix[3][0:4])
        
        if condition1 and condition2 and condition3 and condition4:
            shape_collided = True
        else: 
            shape_collided = False
    else: 
        shape_collided = False

    return bool(shape_collided)
