package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.NoteState
import frc.robot.RobotContainer
import frc.robot.TrunkPose
import frc.robot.commands.trunk.CoastAngleMovePosition
import frc.robot.commands.trunk.GoToPoseAndCoast
import frc.robot.commands.trunk.GoToPoseAndHoldTrunk

class AutoIntakeTrunk : Command() {

    //    var intakePrepCommand = GoToPoseTrunk(TrunkPose.INTAKE_PREP)
//    val coastOutCommand = CoastAngleMovePosition(TrunkPose.INTAKE).andThen(CoastAngleHoldPosition(TrunkPose.INTAKE))
    var intakeOutCommand = GoToPoseAndCoast(TrunkPose.INTAKE, 0.36)
    val stowCommand = CoastAngleMovePosition(TrunkPose.STOW).andThen(GoToPoseAndHoldTrunk(TrunkPose.STOW))
    // val stowCommand = GoToPoseAndHoldTrunk(TrunkPose.STOW)

    var stowing = false


    override fun initialize() {
        RobotContainer.cannonSystem.killShooter()
        intakeOutCommand = GoToPoseAndCoast(TrunkPose.INTAKE, 0.36)


//        intakePrepCommand = GoToPoseTrunk(TrunkPose.INTAKE_PREP)

        RobotContainer.stateMachine.currentTrunkCommand = intakeOutCommand
        stowing = false
    }


    override fun execute() {
        /*if (intakePrepCommand.isFinished && RobotContainer.stateMachine.currentTrunkCommand == intakePrepCommand) {
            RobotContainer.stateMachine.currentTrunkCommand = coastOutCommand
        } else*/ if (RobotContainer.stateMachine.currentTrunkCommand == intakeOutCommand && (RobotContainer.stateMachine.noteState == NoteState.Stored)) {
            RobotContainer.stateMachine.currentTrunkCommand = stowCommand
            stowing = true
        }

    }

    override fun isFinished(): Boolean {
        if (stowing && RobotContainer.trunkSystem.isAtPose) {
            return true
        }
//        println("trunk intake is finished is at pose: ${RobotContainer.trunkSystem.isAtPose}, current trunk command: ${RobotContainer.stateMachine.currentTrunkCommand.name}")
        return false
    }

    override fun end(interrupted: Boolean) {
        if (interrupted) {
            stowCommand.cancel()
//            coastOutCommand.cancel()
//            intakePrepCommand.cancel()
            intakeOutCommand.cancel()

            stowCommand.schedule()
        }

    }
}
