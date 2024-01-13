package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.RobotContainer

class TeleopSwerveDriveCommand : Command() {
    init {
        addRequirements(RobotContainer.swerveDrive)
    }

    override fun execute() {
//        swerveDrive.drive
    }
}