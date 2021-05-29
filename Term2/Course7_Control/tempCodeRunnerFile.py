    x_trajectory = []
    y_trajectory = []
    x_trajectory.append(robot.x)
    y_trajectory.append(robot.y)

    for i in range(n):
        error = robot.y
        alpha = -tau_p * error - tau_d * (error - y_trajectory[i-1])
        robot.move(alpha, speed)
        print('[x=%.5f y=%.5f orient=%.5f]' % (robot.x, robot.y, robot.orientation))
        x_trajectory.append(robot.x)
        y_trajectory.append(robot.y)
    return x_trajectory, y_trajectory