package frc.robot.commands.cannon

import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.NoteState
import frc.robot.RobotContainer

class IntakeCannon() : Command() {
    override fun initialize() {
        RobotContainer.cannonSystem.intake()
    }

    override fun execute() {}

    override fun isFinished(): Boolean {
        return RobotContainer.stateMachine.noteState == NoteState.Stored
    }

    override fun end(interrupted: Boolean) {
        RobotContainer.cannonSystem.killIntake()
    }
}