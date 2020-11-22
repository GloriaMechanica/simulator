import numpy as np
import math


def get_quadrant(vector):
    x = vector[0]
    y = vector[1]
    if x > 0:
        if y > 0:
            q = 1
        else:
            q = 4
    else:
        if y > 0:
            q = 2
        else:
            q = 3
    return q


def model(input_coord, left_params, right_params):
    R = input_coord["radius"]
    G = input_coord["angle"]

    x_left = left_params["x"]
    y_left = left_params["y"]
    r_left = left_params["radius"]

    x_right = right_params["x"]
    y_right = right_params["y"]
    r_right = right_params["radius"]

    x_input = R * math.cos(G)
    y_input = R * math.sin(G)

    # Coefficient vector for quadratic-equation to get parameter for left instance
    coeff_left = [pow(x_input, 2) + pow(y_input, 2),
                  -2 * y_input * (x_input - x_left) + 2 * x_input * (y_input - y_left),
                  pow((x_input - x_left), 2) + pow((y_input - y_left), 2) - pow(r_left, 2)
                  ]

    solution_left = np.roots(coeff_left)
    # print(solution_left)

    # Coefficient vector for quadratic-equation to get parameter for right instance
    coeff_right = [pow(x_input, 2) + pow(y_input, 2),
                   -2 * y_input * (x_input - x_right) + 2 * x_input * (y_input - y_right),
                   pow((x_input - x_right), 2) + pow((y_input - y_right), 2) - pow(r_right, 2)
                   ]

    solution_right = np.roots(coeff_right)
    # print(solution_right)

    if sum(np.iscomplex(solution_right)) or sum(np.iscomplex(solution_left)):
        print("##### ERROR: There is no real solution for the given input #####")
        return False

    intersect_left = [[-y_input * solution_left[0] - x_left + x_input, x_input * solution_left[0] - y_left + y_input],
                      [-y_input * solution_left[1] - x_left + x_input, x_input * solution_left[1] - y_left + y_input]]

    intersect_right = [[-y_input * solution_right[0] - x_right + x_input,
                        x_input * solution_right[0] - y_right + y_input],
                       [-y_input * solution_right[1] - x_right + x_input,
                        x_input * solution_right[1] - y_right + y_input]]

    angle_left = 0
    angle_right = 0

    validate_angle_left = 0
    for h in intersect_left:
        q = get_quadrant(h)
        if q == 1 or q == 4:
            angle_left = math.atan2(h[1], h[0])
            validate_angle_left += 1

    validate_angle_right = 0
    for h in intersect_right:
        q = get_quadrant(h)
        if q == 2 or q == 3:
            angle_right = math.atan2(h[1], -h[0])
            validate_angle_right += 1

    if validate_angle_left != 1 and validate_angle_right != 1:
        print("##### ERROR: Angle out of range #####")
        return False

    print("angles: ", angle_left * 180 / np.pi, angle_right * 180 / np.pi)

    # print("intersections: ", intersect_left, intersect_right)
    return angle_left, angle_right


input_coord = {"radius": 40, "angle": 66 * np.pi / 180}
left_params = {"x": -60, "y": 45, "radius": 35}
right_params = {"x": 60, "y": 35, "radius": 25}

model(input_coord, left_params, right_params)
