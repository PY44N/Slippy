package frc.robot.commands.cannon

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.NoteState
import frc.robot.RobotContainer
import frc.robot.commands.trunk.GoToPoseAndHoldTrunk
import frc.robot.commands.trunk.CalibrateTrunk
import frc.robot.TrunkPose
import frc.robot.Robot
import frc.robot.IntakeState

class AutoSpit : Command() {
    val goToSpitPose = GoToPoseAndHoldTrunk(TrunkPose.CalibrationAngle)

    override fun initialize() {
        RobotContainer.cannonSystem.killShooter()

        RobotContainer.stateMachine.currentTrunkCommand = goToSpitPose
    }

    override fun execute() {
        if (RobotContainer.trunkSystem.isAtPose && RobotContainer.stateMachine.intakeState != IntakeState.Spitting) {
            RobotContainer.cannonSystem.spit()
        }
    }

    override fun isFinished(): Boolean {
        return RobotContainer.stateMachine.noteState == NoteState.Empty
    }

    override fun end(interrupted: Boolean) {
        RobotContainer.stateMachine.currentTrunkCommand = GoToPoseAndHoldTrunk(TrunkPose.STOW)
        RobotContainer.cannonSystem.killIntake()
    }
}