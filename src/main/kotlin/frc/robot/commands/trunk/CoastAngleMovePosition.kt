package frc.robot.commands.trunk

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.RobotContainer
import frc.robot.TrunkPose

class CoastAngleMovePosition(val pose: TrunkPose): Command() {
    override fun execute() {
        RobotContainer.trunkSystem.io.rotationBrake = false

        val elevatorPercent = RobotContainer.trunkSystem.calculatePositionOut(pose.position)
        RobotContainer.trunkSystem.io.setElevatorSpeed(elevatorPercent)
    }

    override fun isFinished(): Boolean {

        return RobotContainer.trunkSystem.checkAtPosition(pose.position)
    }

    override fun end(interrupted: Boolean) {
        if (interrupted == false) {
            RobotContainer.trunkSystem.isAtPose = true
        }
    }
}