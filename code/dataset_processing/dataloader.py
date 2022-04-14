class Dataloader:

    def __init__(self):
        raise NotImplementedError

    def query(self, start_point, robot_velocity, time_interval, time_resolution):
        """
        Given a robot start point (x, y, z), relative velocity (dz, dy, dz), and a time interval,
        return a list of points that the robot travels through during the time interval,
        giving one point for every time_resolution seconds of travel.
        """
        raise NotImplementedError