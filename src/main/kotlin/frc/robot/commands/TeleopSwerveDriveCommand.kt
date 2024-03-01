package frc.robot.commands

import MiscCalculations.calculateDeadzone
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.DriveState
import frc.robot.RobotContainer
import frc.robot.constants.DriveConstants

class TeleopSwerveDriveCommand : Command() {
    init {
//        addRequirements(RobotContainer.swerveSystem)
    }

    override fun initialize() {
        RobotContainer.stateMachine.driveState = DriveState.Teleop
    }

    fun teleopTranslationAutoTwist(desiredAngle: Double) {
        if (RobotContainer.stateMachine.driveState != DriveState.TranslationTeleop) {
            return
        }
    }




    override fun execute() {
        val twoJoysticks=true
        val twist = if (twoJoysticks) {-RobotContainer.leftJoystick.x} else {RobotContainer.rightJoystick.twist}

        val throttle = RobotContainer.swerveSystem.calculateJoyThrottle(RobotContainer.leftJoystick.throttle)


        if (RobotContainer.rightJoystick.button(2).asBoolean) {
            RobotContainer.swerveSystem.swerveDrive.zeroGyro()
        }

        if (RobotContainer.stateMachine.driveState != DriveState.Teleop) {
            return
        }

        RobotContainer.swerveSystem.drive(
                RobotContainer.swerveSystem.calculateJoyTranslation(RobotContainer.rightJoystick.x, RobotContainer.rightJoystick.y, throttle, DriveConstants.TELEOP_DEADZONE_X, DriveConstants.TELEOP_DEADZONE_Y),
                calculateDeadzone(twist, DriveConstants.TELEOP_DEADZONE_TWIST) * throttle * DriveConstants.MAX_ANGLE_SPEED,
                true
        )
    }
}