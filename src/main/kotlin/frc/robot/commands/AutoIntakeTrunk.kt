package frc.robot.commands

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.robot.NoteState
import frc.robot.RobotContainer
import frc.robot.TrunkPose
import frc.robot.commands.cannon.HalfSpitCannon
import frc.robot.commands.cannon.IntakeCannon
import frc.robot.commands.trunk.CoastAngleMovePosition
import frc.robot.commands.trunk.CoastAngleHoldPosition
import frc.robot.commands.trunk.GoToPoseAndHoldTrunk
import frc.robot.commands.trunk.GoToPoseTrunk

class AutoIntakeTrunk : Command() {

    val intakePrepCommand = GoToPoseTrunk(TrunkPose.INTAKE_PREP)
    val coastOutCommand = CoastAngleMovePosition(TrunkPose.INTAKE).andThen(CoastAngleHoldPosition(TrunkPose.INTAKE))
    val stowCommand = CoastAngleMovePosition(TrunkPose.STOW).andThen(GoToPoseAndHoldTrunk(TrunkPose.STOW))
    // val stowCommand = GoToPoseAndHoldTrunk(TrunkPose.STOW)

    override fun initialize() {
        RobotContainer.cannonSystem.killShooter()

        RobotContainer.stateMachine.currentTrunkCommand = intakePrepCommand
    }


    override fun execute() {
        if (intakePrepCommand.isFinished && RobotContainer.stateMachine.currentTrunkCommand == intakePrepCommand) {
            RobotContainer.stateMachine.currentTrunkCommand = coastOutCommand
        } else if (RobotContainer.stateMachine.currentTrunkCommand == coastOutCommand && (RobotContainer.stateMachine.noteState == NoteState.Stored)) {
            RobotContainer.stateMachine.currentTrunkCommand = stowCommand
        }
    } 

    override fun isFinished(): Boolean {
        if (RobotContainer.stateMachine.currentTrunkCommand == stowCommand && RobotContainer.trunkSystem.isAtPose) {
            println("trunk intake finished")
            return true
        }
        return false
    }

    override fun end(interrupted: Boolean) {
        if (interrupted) {
            stowCommand.cancel()
            coastOutCommand.cancel()
            intakePrepCommand.cancel()
        }
    }
}
