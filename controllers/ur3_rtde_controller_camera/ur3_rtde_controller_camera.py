from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())
camera = robot.getDevice('cam1')
camera.enable(timestep)

while robot.step(timestep) != -1:
    image = camera.getImage()
    # Bildverarbeitung hier
