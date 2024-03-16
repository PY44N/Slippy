package frc.robot.commands.cannon

import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.NoteState
import frc.robot.RobotContainer
import frc.robot.util.Timer

class HalfSpitCannon() : Command() {
    val timer = Timer()

    override fun initialize() {
        RobotContainer.cannonSystem.spit()
        timer.reset()
        timer.start()
    }

    override fun execute() {}

    override fun isFinished(): Boolean {
        return timer.hasElapsed(0.15)
    }

    override fun end(interrupted: Boolean) {
        RobotContainer.cannonSystem.killIntake()
        timer.reset()
    }
}