package frc.robot.subsystems.trunk

import cshcyberhawks.lib.requests.Request
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.constants.TrunkConstants

class Pivot(private val io: PivotIO) : SubsystemBase() {
    private var desiredRotation = TrunkConstants.STOW_ANGLE
        set(value) {
            io.setDesiredRotation(value)
            field = value
        }

    private fun atRotation(pos: Double): Boolean =
        MiscCalculations.appxEqual(pos, io.getRotation(), TrunkConstants.ELEVATOR_DEADZONE)

    fun coastRequest() = Request.withAction { io.setCoast(true) }
    fun brakeRequest() = Request.withAction { io.setCoast(false) }

    fun travelRequest() = object : Request() {
        override fun execute() {
            desiredRotation = TrunkConstants.SAFE_TRAVEL_ANGLE
        }

        override fun isFinished(): Boolean = atRotation(TrunkConstants.SAFE_TRAVEL_ANGLE)
    }

    fun stowRequest() = object : Request() {
        override fun execute() {
            desiredRotation = TrunkConstants.STOW_ANGLE
        }

        override fun isFinished(): Boolean = atRotation(TrunkConstants.STOW_ANGLE)
    }

    override fun periodic() {
        io.update()
    }
}