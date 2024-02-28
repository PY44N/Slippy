package frc.robot.commands

import MiscCalculations.calculateDeadzone
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.DriveState
import frc.robot.Robot
import frc.robot.RobotContainer
import frc.robot.constants.DriveConstants

class TeleopSwerveDriveCommand : Command() {
    init {
        addRequirements(RobotContainer.swerveSystem)
    }

    override fun initialize() {
        RobotContainer.stateMachine.driveState = DriveState.Teleop
    }

    override fun execute() {
        val deadzoneX=.15
        val deadzoneY=.15
        val deadzoneTwist=.15
        val twoJoysticks=true

        val twist = if (twoJoysticks) {-RobotContainer.leftJoystick.x} else {RobotContainer.rightJoystick.twist}


        //Milan: trust me bro this'll work totally definitely please don't question it
        val throttle = ((RobotContainer.rightJoystick.throttle * -1) + 1) / 2


        if (RobotContainer.rightJoystick.button(2).asBoolean) {
            RobotContainer.swerveSystem.swerveDrive.zeroGyro()
        }

        if (RobotContainer.stateMachine.driveState != DriveState.Teleop) {
            return
        }

        RobotContainer.swerveSystem.drive(
                Translation2d(
                        -calculateDeadzone(RobotContainer.rightJoystick.y,deadzoneX) * DriveConstants.MAX_SPEED * throttle,
                        -calculateDeadzone(RobotContainer.rightJoystick.x,deadzoneY) * DriveConstants.MAX_SPEED * throttle
                ),
                calculateDeadzone(twist, deadzoneTwist) * throttle * DriveConstants.MAX_ANGLE_SPEED,
                true
        )
    }
}