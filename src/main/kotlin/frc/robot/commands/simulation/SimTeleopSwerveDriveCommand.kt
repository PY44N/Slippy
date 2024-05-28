package frc.robot.commands.simulation

import MiscCalculations.calculateDeadzone
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.DriveState
import frc.robot.RobotContainer
import frc.robot.constants.DriveConstants
import frc.robot.util.ControllerUtil

class SimTeleopSwerveDriveCommand : Command() {
    var disableFieldOrientation = false

    override fun initialize() {
        RobotContainer.stateMachine.driveState = DriveState.Teleop

        RobotContainer.xboxController.pov(0).onTrue(Commands.runOnce({
            disableFieldOrientation = true
        }))
        RobotContainer.xboxController.pov(0).onFalse(Commands.runOnce({
            disableFieldOrientation = false
        }))
    }

    override fun execute() {
        val translation = ControllerUtil.calculateJoyTranslation(
            -RobotContainer.xboxController.leftX,
            -RobotContainer.xboxController.leftY,
            0.5,
            DriveConstants.TELEOP_DEADZONE_X,
            DriveConstants.TELEOP_DEADZONE_Y
        )

        val twist = -calculateDeadzone(
            RobotContainer.xboxController.rightX,
            DriveConstants.TELEOP_DEADZONE_TWIST_TWO_JOY
        ) * 0.5 * DriveConstants.MAX_ANGLE_SPEED

        if (disableFieldOrientation) {
            RobotContainer.swerveSystem.applyRobotRelativeDriveRequest(translation.x, translation.y, twist)
                .execute()
        } else {
            RobotContainer.swerveSystem.applyDriveRequest(translation.x, translation.y, twist).execute()
        }
    }
}