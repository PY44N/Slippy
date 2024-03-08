package frc.robot.commands.cannon

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.NoteState
import frc.robot.RobotContainer
import frc.robot.TrunkPosition

class AutoShootCommand : Command() {


    override fun initialize() {
        RobotContainer.cannonSystem.shoot()
        RobotContainer.stateMachine.targetTrunkPose = TrunkPosition.SPEAKER
    }

    override fun execute() {
        if (RobotContainer.stateMachine.shooterReady) {
            RobotContainer.cannonSystem.feed()
//            println("feeding")
        }
        SmartDashboard.putBoolean("shooter ready", RobotContainer.stateMachine.shooterReady )


//        SmartDashboard.putBoolean("shooter ready", false)
    }

    override fun isFinished(): Boolean {
        return RobotContainer.stateMachine.noteState == NoteState.Empty
    }

    override fun end(interrupted: Boolean) {
        RobotContainer.cannonSystem.killShooter()
        RobotContainer.cannonSystem.killIntake()
        RobotContainer.stateMachine.targetTrunkPose = TrunkPosition.STOW
    }
}
