package frc.robot.commands.cannon

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.NoteState
import frc.robot.RobotContainer
import frc.robot.TrunkPosition
import frc.robot.TrunkState

class AutoIntake : Command() {
    override fun initialize() {
        RobotContainer.cannonSystem.intake()
        RobotContainer.cannonSystem.killShooter()
        RobotContainer.stateMachine.targetTrunkPose = TrunkPosition.INTAKE
    }

    override fun execute() {
//        if (RobotContainer.stateMachine.noteState == NoteState.Intaking) {
//            RobotContainer.cannonSystem.feed()
//        }
    }

    override fun isFinished(): Boolean = RobotContainer.stateMachine.noteState == NoteState.Stored

    override fun end(interrupted: Boolean) {
        RobotContainer.cannonSystem.killIntake()
        RobotContainer.stateMachine.targetTrunkPose = TrunkPosition.STOW
    }
}