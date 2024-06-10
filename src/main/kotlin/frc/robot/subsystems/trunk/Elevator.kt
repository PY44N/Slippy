package frc.robot.subsystems.trunk

import cshcyberhawks.lib.requests.Prerequisite
import cshcyberhawks.lib.requests.Request
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.constants.TrunkConstants

class Elevator(private val io: ElevatorIO) : SubsystemBase() {
    private var desiredPosition = TrunkConstants.STOW_POSITION
        set(value) {
            io.setDesiredPosition(value)
            field = value
        }

    private fun atPosition(pos: Double): Boolean =
        MiscCalculations.appxEqual(pos, io.getPosition(), TrunkConstants.ELEVATOR_DEADZONE)

    fun elevatorBelow(position: Double) = Prerequisite.withCondition { io.getPosition() <= position }

    fun stowRequest() = object : Request() {
        override fun execute() {
            desiredPosition = TrunkConstants.STOW_POSITION
        }

        override fun isFinished(): Boolean = atPosition(TrunkConstants.STOW_POSITION)
    }

    fun intakeRequest() = object : Request() {
        override fun execute() {
            desiredPosition = TrunkConstants.INTAKE_POSITION
        }

        override fun isFinished(): Boolean = atPosition(TrunkConstants.INTAKE_POSITION)
    }

    override fun periodic() {
        io.update()
    }
}