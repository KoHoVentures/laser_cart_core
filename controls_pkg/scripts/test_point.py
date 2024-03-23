import numpy as np

def interpolate_points(point1, point2, num_intermediate_points):
    interpolated_points = []
    for i in range(1, num_intermediate_points + 1):
        alpha = i / (num_intermediate_points + 1)
        interpolated_point = [
            point1[0] * (1 - alpha) + point2[0] * alpha,
            point1[1] * (1 - alpha) + point2[1] * alpha,
            point1[2] * (1 - alpha) + point2[2] * alpha
        ]
        interpolated_points.append(interpolated_point)
    return interpolated_points

def populate_points_fixed_threshold(points_list, threshold):
    new_points = []
    for i in range(len(points_list) - 1):
        point1 = points_list[i]
        new_points.append(point1)  # Include original point
        point2 = points_list[i + 1]
        distance = max(abs(point2[0] - point1[0]), abs(point2[1] - point1[1]), abs(point2[2] - point1[2]))
        num_intermediate_points = int(distance / threshold)
        if num_intermediate_points > 0:
            intermediate_points = interpolate_points(point1, point2, num_intermediate_points)
            new_points.extend(intermediate_points)
    new_points.append(points_list[-1])  # Include last original point
    return new_points

points_list = [[10, 20, 10], [50, 50, 20], [100, 30, 10]]
threshold = 15
populated_points = populate_points_fixed_threshold(points_list, threshold)
print("Original points list:", points_list)
print("Populated points list:", populated_points)
