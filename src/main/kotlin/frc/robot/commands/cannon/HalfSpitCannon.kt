package frc.robot.commands.cannon

import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.NoteState
import frc.robot.RobotContainer
import frc.robot.util.Timer
import kotlin.math.abs

class HalfSpitCannon() : Command() {
    val timer = Timer()
    var intakeStartPos = RobotContainer.cannonSystem.io.getIntakePosition()

    override fun initialize() {
        RobotContainer.cannonSystem.spit()
        timer.reset()
        timer.start()
        intakeStartPos = RobotContainer.cannonSystem.io.getIntakePosition()
    }

    override fun execute() {
        println("Half Spitting")
    }

    override fun isFinished(): Boolean {
        return intakeStartPos - RobotContainer.cannonSystem.io.getIntakePosition() >= 3.0
    }

    override fun end(interrupted: Boolean) {
        RobotContainer.cannonSystem.killIntake()
        timer.reset()
    }
}