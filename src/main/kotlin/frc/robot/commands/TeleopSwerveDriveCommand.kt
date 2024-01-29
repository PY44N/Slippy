package frc.robot.commands

import MiscCalculations.calculateDeadzone
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.RobotContainer
import frc.robot.constants.DriveConstants

class TeleopSwerveDriveCommand : Command() {
    init {
        addRequirements(RobotContainer.swerveSystem)
    }

    override fun execute() {
//        swerveDrive.drive
        val deadzoneX=.15
        val deadzoneY=.15
        val deadzoneTwist=.15
        val twoJoysticks=false

        val twist = if (twoJoysticks) {-RobotContainer.leftJoystick.y} else {RobotContainer.rightJoystick.twist}

        RobotContainer.swerveSystem.drive(
                Translation2d(
                        /**(if (abs(RobotContainer.rightJoystick.x) > 0.15) {
                            val inSpeed =
                                    if (RobotContainer.rightJoystick.x < 0.0) RobotContainer.rightJoystick.x + .15 else RobotContainer.rightJoystick.x - .15
                            (inSpeed) * DriveConstants.MAX_SPEED
                        } else 0.0),
                        (if (abs(RobotContainer.rightJoystick.y) > 0.15) {
                            val inSpeed =
                                    if (RobotContainer.rightJoystick.y < 0.0) RobotContainer.rightJoystick.y + .15 else RobotContainer.rightJoystick.y - .15
                            (-inSpeed) * DriveConstants.MAX_SPEED
                        } else 0.0)**/
                        calculateDeadzone(RobotContainer.rightJoystick.x,deadzoneX) * DriveConstants.MAX_SPEED,
                        (-calculateDeadzone(RobotContainer.rightJoystick.y,deadzoneY)) * DriveConstants.MAX_SPEED
                ),
                calculateDeadzone(twist, deadzoneTwist),
                true
        )
    }
}