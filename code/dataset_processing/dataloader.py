class Dataloader:

    def __init__(self):
        raise NotImplementedError

    def query(self, start_point, robot_velocity, time_interval, time_resolution, current_time=0):
        """
        Given a robot start point (x, y, z), relative velocity (dx, dy, dz), and a time interval,
        return a list of points that the robot travels through during the time interval,
        giving one point for every time_resolution seconds of travel.
        Ex: query((0, 0, 0), (0, 0, -1), 1, 0.1) = [(1, 0, 0), (2, 0, 0), (3, 0, 0)]
        """
        raise NotImplementedError