package frc.robot.commands

import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.robot.RobotContainer

class ResetSwerveFieldForward: InstantCommand() {
    init {
        addRequirements(RobotContainer.swerveSystem)
    }
    override fun execute() {
        RobotContainer.swerveSystem.swerveDrive.zeroGyro()

        super.execute()
    }
}