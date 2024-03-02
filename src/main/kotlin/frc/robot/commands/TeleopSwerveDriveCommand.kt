package frc.robot.commands

import MiscCalculations.calculateDeadzone
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.robot.DriveState
import frc.robot.Robot
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
        val twoJoysticks = true
        val twistInput = if (twoJoysticks) {
            -RobotContainer.leftJoystick.x
        } else {
            RobotContainer.rightJoystick.twist
        }

        val throttle = RobotContainer.swerveSystem.calculateJoyThrottle(RobotContainer.leftJoystick.throttle)


        if (RobotContainer.rightJoystick.button(2).asBoolean) {
            RobotContainer.swerveSystem.zeroGyro()
        }

        if (RobotContainer.stateMachine.driveState != DriveState.Teleop) {
            return
        }

        val translation = RobotContainer.swerveSystem.calculateJoyTranslation(RobotContainer.rightJoystick.x, RobotContainer.rightJoystick.y, throttle, DriveConstants.TELEOP_DEADZONE_X, DriveConstants.TELEOP_DEADZONE_Y)
        SmartDashboard.putNumber("drive in x", translation.x)
        SmartDashboard.putNumber("drive in y", translation.y)
        val twist = calculateDeadzone(twistInput, DriveConstants.TELEOP_DEADZONE_TWIST) * throttle * DriveConstants.MAX_ANGLE_SPEED
//        println("driving")
        RobotContainer.swerveSystem.driveTrain.applyRequest {
            RobotContainer.swerveSystem.drive.withVelocityX(translation.x)
                    .withVelocityY(translation.y)
                    .withRotationalRate(twist)
        }.execute()
    }
}
