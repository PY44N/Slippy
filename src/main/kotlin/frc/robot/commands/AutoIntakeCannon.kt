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
import frc.robot.commands.trunk.GoToPoseAndHoldTrunk
import frc.robot.commands.trunk.GoToPoseTrunk

class AutoIntakeCannon : Command() {

    var intakeCommand = SequentialCommandGroup(
        IntakeCannon(),
        HalfSpitCannon(),
        WaitCommand(.05),
        IntakeCannon(),
    )

    override fun initialize() {
        RobotContainer.cannonSystem.killShooter()
        intakeCommand.schedule()
    }

    private fun cancelCommand() {
        intakeCommand.cancel()
    }

    override fun execute() {
    }

    override fun isFinished(): Boolean {
        return intakeCommand.isFinished
    }

    override fun end(interrupted: Boolean) {
        intakeCommand.cancel()
    }
}
