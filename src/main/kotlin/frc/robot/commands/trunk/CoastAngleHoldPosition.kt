package frc.robot.commands.trunk

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.RobotContainer
import frc.robot.TrunkPose

class CoastAngleHoldPosition(val pose: TrunkPose) : Command() {
    override fun execute() {
        RobotContainer.trunkSystem.io.rotationBrake = false
        RobotContainer.trunkSystem.isAtPose = false

//        SmartDashboard.putNumber("Coast Angle Move Position", pose.position)
        val elevatorPercent = RobotContainer.trunkSystem.calculatePositionOut(pose.position)
        RobotContainer.trunkSystem.io.setElevatorSpeed(elevatorPercent)
        RobotContainer.trunkSystem.io.setRotationVoltage(0.0)

        if (RobotContainer.trunkSystem.checkAtPosition(pose.position)) {
            RobotContainer.trunkSystem.isAtPose = true
        }
    }

    override fun isFinished(): Boolean {
        return false
    }

    override fun end(interrupted: Boolean) {
        RobotContainer.trunkSystem.io.rotationBrake = true
        if (interrupted == false) {
            RobotContainer.trunkSystem.isAtPose = true
        }
    }
}
