package frc.robot.commands.cannon

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.NoteState
import frc.robot.RobotContainer
import frc.robot.TrunkPosition

class AutoAmp : Command() {
    override fun initialize() {
//        RobotContainer.cannonSystem.ampSpit()
        RobotContainer.cannonSystem.killShooter()
        RobotContainer.stateMachine.targetTrunkPose = TrunkPosition.AMP

    }

    override fun execute() {
        if (!RobotContainer.trunkSystem.isMoving) {
            RobotContainer.cannonSystem.ampSpit()
        }
    }

    override fun isFinished(): Boolean {
        return RobotContainer.stateMachine.noteState == NoteState.Empty
    }

    override fun end(interrupted: Boolean) {
        RobotContainer.cannonSystem.killIntake()
        println("amp end")
    }
}