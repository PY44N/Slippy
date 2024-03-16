package frc.robot.commands.trunk

import edu.wpi.first.wpilibj2.command.Command

class ServoLockElevator : Command() {
    override fun initialize() {
        cancel()
    }

    override fun execute() {
    }

    override fun isFinished(): Boolean {
        return true
    }

    override fun end(interrupted: Boolean) {
    }
}
