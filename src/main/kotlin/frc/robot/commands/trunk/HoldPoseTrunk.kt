package frc.robot.commands.trunk

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.RobotContainer
import frc.robot.TrunkPose

class HoldPoseTrunk(val pose: TrunkPose) : Command() {
    var currentTargetPosition = pose.position
    var currentTargetRotation = pose.angle

    override fun execute() {
        val rotationVolts = RobotContainer.trunkSystem.calculateRotationOut(currentTargetRotation)

        RobotContainer.trunkSystem.io.setRotationVoltage(rotationVolts)

        val elevatorPercent = RobotContainer.trunkSystem.calculatePositionOut(currentTargetPosition)

        RobotContainer.trunkSystem.io.setElevatorSpeed(elevatorPercent)
    }

    override fun isFinished(): Boolean {
        return false
    }
}