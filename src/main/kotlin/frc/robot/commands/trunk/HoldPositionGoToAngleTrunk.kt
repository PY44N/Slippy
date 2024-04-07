package frc.robot.commands.trunk

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.RobotContainer
import frc.robot.TrunkPose

class HoldPositionGoToAngleTrunk(val pose: TrunkPose) : Command() {

    var desiredAngle: Double = pose.angle
        set(value) {
            RobotContainer.trunkSystem.setDesiredRotation(value)
            field = value
        }

    override fun initialize() {
        RobotContainer.trunkSystem.isAtPose = false
        RobotContainer.trunkSystem.setDesiredRotation(desiredAngle)
        RobotContainer.trunkSystem.setDesiredPosition(pose.position)
    }

    override fun execute() {
        val rotationVolts = RobotContainer.trunkSystem.calculateRotationOut(desiredAngle)

        RobotContainer.trunkSystem.io.setRotationVoltage(rotationVolts)

        val elevatorPercent = RobotContainer.trunkSystem.calculatePositionOut(pose.position)

        RobotContainer.trunkSystem.io.setElevatorSpeed(elevatorPercent)

        if (RobotContainer.trunkSystem.checkAtPose(desiredAngle, pose.position)) {
            RobotContainer.trunkSystem.isAtPose = true
        }
    }

    override fun isFinished(): Boolean {
        return false
    }
}
