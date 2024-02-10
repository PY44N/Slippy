from klampt import IKObjective, IKSolver, WorldModel, vis
from klampt.model import ik

world = WorldModel()
world.loadElement("models/robot-urdf/robot.urdf")

vis.add("world", world)
vis.run()