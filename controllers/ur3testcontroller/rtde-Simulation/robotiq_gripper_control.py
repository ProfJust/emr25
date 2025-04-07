class RobotiqGripper:
    def __init__(self, rtde_control):
        self.rtde = rtde_control

    def activate(self):
        return self.rtde.send_gripper_command("activate")

    def set_force(self, force_percent):
        return self.rtde.send_gripper_command("set_force", force_percent)

    def set_speed(self, speed_percent):
        return self.rtde.send_gripper_command("set_speed", speed_percent)

    def open(self):
        return self.rtde.send_gripper_command("OpenGripper")

    def close(self):
        return self.rtde.send_gripper_command("CloseGripper")

    def move(self, opening_mm):
        return self.rtde.send_gripper_command("move", opening_mm)
