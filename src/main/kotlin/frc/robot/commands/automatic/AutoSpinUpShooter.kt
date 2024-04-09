package frc.robot.commands.automatic

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.RobotContainer
import frc.robot.ShooterState

class AutoSpinUpShooter() : Command() {
    override fun initialize() {
        RobotContainer.stateMachine.shooterState = ShooterState.Shooting
    }
}