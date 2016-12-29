from copy import deepcopy


#
# This method calculates the optimal parameters for determining what amount of nodes to inject into the path
# to meet the time restraint. This approach uses an iterative process to inject and smooth, yielding more desirable
# results for the final smooth path.
# Big O: Constant Time
def injection_counter_to_steps(path_length, max_time_to_complete_, time_step):
    first = 0
    second = 0
    third = 0

    old_points_total = 0

    ret = []

    total_points = max_time_to_complete_ / time_step

    if total_points < 100:
        points_first = None
        points_total = None
        for i in range(4, 6):
            for j in range(1, 8):
                points_first = i * (path_length - 1) + path_length
                points_total = (j * (points_first - 1) + points_first)

                if old_points_total < points_total <= total_points:
                    first = i
                    second = j
                    old_points_total = points_total
    else:
        points_first = None
        points_second = None
        points_total = None
        for i in range(1, 6):
            for j in range(1, 7):
                for k in range(1, 8):
                    points_first = i * (path_length - 1) + path_length;
                    points_second = (j * (points_first - 1) + points_first);
                    points_total = (k * (points_second - 1) + points_second);

                    if points_total <= total_points:
                        first = i
                        second = j
                        third = k

    return [first, second, third]


# Method up-samples the Path by linear injection. The result providing more waypoints along the path.
# BigO: Order N * injection
def inject(orig, num_to_inject):
    more_points = []
    length = len(orig) + (num_to_inject * (len(orig) - 1))
    # loop through original array
    for i in range(0, length):
        tmp = []
        for dim in orig[0]:
            tmp.append(0)
        more_points.append(tmp)

    index = 0

    for i in range(0, len(orig) - 1):
        for j in range(0, len(orig[i])):
            more_points[index][j] = orig[i][j]

        index += 1

        for j in range(1, num_to_inject + 1):
            for k in range(0, len(orig[0])):
                # calculate intermediate x points between j and j+1 original points
                more_points[index][k] = j * ((orig[i + 1][k] - orig[i][k]) / (num_to_inject + 1)) + orig[i][k]
            index += 1

    # copy last
    for j in range(0, len(orig[-1])):
        more_points[-1][j] = orig[-1][j]

    return more_points


def smoother(orig, weight_data, weight_smooth, tolerance):
        # copy path
        new_path = deepcopy(orig)

        change = tolerance
        while change >= tolerance:
            change = 0.0
            for i in range(0, len(new_path) - 1):
                for j in range(0, len(orig[i])):
                    aux = new_path[i][j]
                    new_path[i][j] += weight_data * (orig[i][j] - new_path[i][j]) + weight_smooth * (new_path[i - 1][j] + new_path[i + 1][j] - (2.0 * new_path[i][j]))
                    change += abs(aux-new_path[i][j])
        return new_path


def smooth_path(orig, total_time, time_step):
    # smoothing parameters
    path_alpha = 0.7
    path_beta = 0.3
    path_tolerance = 0.0000001

    # smooth path that will be returned
    path = []
    injections = injection_counter_to_steps(len(orig), total_time, time_step)
    for i in injections:
        if injections.index(i) == 0:
            path = inject(orig, i)
            path = smoother(path, path_alpha, path_beta, path_tolerance)
        else:
            path = inject(path, i)
            path = smoother(path, path_alpha, path_beta, path_tolerance)

    return path


def test():
    points = []

    point1 = [1, 2]
    point2 = [2, 7]
    point3 = [4, 7]
    point4 = [6, 9]
    point5 = [10, 11]

    points.append(point1)
    points.append(point2)
    points.append(point3)
    points.append(point4)
    points.append(point5)

    total_time = 15
    time_step = .1
    path = smooth_path(points, total_time, time_step)

    for point in path:
        print(point)


if __name__ == "__main__":
    test()
