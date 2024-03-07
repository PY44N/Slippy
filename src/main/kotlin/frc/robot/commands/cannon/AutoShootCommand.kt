package frc.robot.commands.cannon

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.NoteState
import frc.robot.RobotContainer
import frc.robot.TrunkPosition

class AutoShootCommand : Command() {

    var hasShooterBeenReady: Boolean = false
    override fun initialize() {
        RobotContainer.cannonSystem.shoot()
        RobotContainer.stateMachine.targetTrunkPose = TrunkPosition.SPEAKER
    }

    override fun execute() {
        if (RobotContainer.stateMachine.shooterReady || hasShooterBeenReady) {
            RobotContainer.cannonSystem.feed()
            hasShooterBeenReady = true
            println("feeding")
        }
        SmartDashboard.putBoolean("shooter ready", RobotContainer.stateMachine.shooterReady || hasShooterBeenReady)

//        SmartDashboard.putBoolean("shooter ready", false)
    }

    override fun isFinished(): Boolean {
        return RobotContainer.stateMachine.noteState == NoteState.Empty
    }

    override fun end(interrupted: Boolean) {
        RobotContainer.cannonSystem.killShooter()
        RobotContainer.cannonSystem.killIntake()
    }
}
