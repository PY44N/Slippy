package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.RobotContainer
import frc.robot.TrunkPose
import frc.robot.commands.trunk.GoToPoseAndHoldTrunk
import frc.robot.commands.trunk.GoToPoseTrunk
import frc.robot.util.Timer

class AutoAmp : Command() {
    val startTimer = Timer()
    val endTimer = Timer()

    override fun initialize() {
        RobotContainer.cannonSystem.killShooter()

        RobotContainer.stateMachine.currentTrunkCommand =
            GoToPoseTrunk(TrunkPose.AMP_GOING).andThen(GoToPoseAndHoldTrunk(TrunkPose.AMP))
        RobotContainer.actuallyDoAmp = false
        endTimer.reset()
        startTimer.reset()
    }

    override fun execute() {
//        if (RobotContainer.xboxController.start().asBoolean) {
//            RobotContainer.cannonSystem.ampSpit()
//        } else {
//            RobotContainer.cannonSystem.killIntake()
//        }
        if (RobotContainer.actuallyDoAmp && !startTimer.isRunning) {
            startTimer.start()
            RobotContainer.cannonSystem.ampSpit()
        }

        if (startTimer.hasElapsed(0.3) && !endTimer.isRunning) {
            endTimer.start()
            RobotContainer.stateMachine.currentTrunkCommand = GoToPoseAndHoldTrunk(TrunkPose.STOW)
        }
    }

    override fun isFinished(): Boolean {
//        return RobotContainer.stateMachine.noteState == NoteState.Empty && RobotContainer.stateMachine.intakeState != IntakeState.AmpSpitting
        return endTimer.hasElapsed(0.75)
    }

    override fun end(interrupted: Boolean) {
        endTimer.reset()
        RobotContainer.cannonSystem.killIntake()
        RobotContainer.actuallyDoAmp = false
    }
}
