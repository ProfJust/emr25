
class RobotiqGripper:
    def __init__(self, rtde_control):
        self.rtde = rtde_control

    def activate(self):
        return self.rtde._send_command("activate")

    def open(self):
        return self.rtde._send_command("openGripper")

    def close(self):
        return self.rtde._send_command("closeGripper")

    def move(self, value):  # mm
        return self.rtde._send_command("move", {"value": value})

    def set_force(self, value):
        return self.rtde._send_command("set_force", {"value": value})

    def set_speed(self, value):
        return self.rtde._send_command("set_speed", {"value": value})
