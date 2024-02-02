package frc.robot.subsystems

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.SubsystemBase

class IntakeShooterSystem : SubsystemBase() {

    fun setDesiredPosition(angle: Rotation2d, distance: Double) {

    }

    fun setDesiredSpeedRPM(left: Double, right: Double) {

    }

    fun setDesiredSpeedPower(left: Double, right: Double) {
    }

    fun setDesiredPositionProportion(percent: Double) {
    }

    override fun periodic() {
    }

    override fun simulationPeriodic() {
    }

    companion object {
        const val MAX_HEIGHT_M = 1.0;
        const val MIN_HEIGHT_M = .5;
    }
}
