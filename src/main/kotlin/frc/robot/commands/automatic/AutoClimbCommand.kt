package frc.robot.commands.automatic

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.RobotContainer

class AutoClimbCommand : Command() {
    override fun initialize() {
    }

    override fun execute() {

    }

    override fun end(interrupted: Boolean) {
        SmartDashboard.putBoolean("Pulldown Climb?", false)
    }
}