package frc.robot.subsystems

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.SubsystemBase

class GUNSystem : SubsystemBase() {

    fun goToPosition(angle: Rotation2d, elevatorHeight: Double) {

    }

    fun setSpeed(left: Double, right: Double) {
    }

    fun shoot(angle: Rotation2d, elevatorHeight: Double, leftPower: Double, rightPower: Double) {
        goToPosition(angle, elevatorHeight)
        setSpeed(leftPower, rightPower)
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
