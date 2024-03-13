package frc.robot.commands.cannon

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.NoteState
import frc.robot.RobotContainer
import frc.robot.TrunkPose
import frc.robot.commands.trunk.CoastAngleMovePosition
import frc.robot.commands.trunk.GoToPoseTrunk
import frc.robot.commands.trunk.CoastTrunk

class AutoIntake : Command() {

    var hasIntake: Boolean = false
    var hasAlmostSpit: Boolean = false

    override fun initialize() {
        RobotContainer.cannonSystem.intake()
        RobotContainer.cannonSystem.killShooter()
        hasIntake = false
        hasAlmostSpit = false

        RobotContainer.stateMachine.currentTrunkCommand = GoToPoseTrunk(TrunkPose.INTAKE_PREP)
    }

    override fun execute() {

        if (RobotContainer.stateMachine.trunkReady) {
            RobotContainer.stateMachine.currentTrunkCommand = CoastAngleMovePosition(TrunkPose.INTAKE)
        }

        if (RobotContainer.stateMachine.noteState == NoteState.Stored && !hasIntake) {
            RobotContainer.stateMachine.currentTrunkCommand = CoastAngleMovePosition(TrunkPose.STOW).andThen(GoToPoseTrunk(TrunkPose.STOW))
            RobotContainer.cannonSystem.killIntake()
            RobotContainer.cannonSystem.spit()
            hasIntake = true
            println("spitting back out")
        }
        if (RobotContainer.stateMachine.noteState == NoteState.Intaking && hasIntake && !hasAlmostSpit) {
            RobotContainer.cannonSystem.intake()
            hasAlmostSpit = true
            println("intaking back up")
        }

        if (hasAlmostSpit) {
            RobotContainer.cannonSystem.noteEntryTime = -1.0
        }
    }

    override fun isFinished(): Boolean = RobotContainer.stateMachine.noteState == NoteState.Stored && hasIntake && hasAlmostSpit

    override fun end(interrupted: Boolean) {
        RobotContainer.cannonSystem.killIntake()
    }
}