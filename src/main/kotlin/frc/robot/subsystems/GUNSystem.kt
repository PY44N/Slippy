package frc.robot.subsystems

import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.SubsystemBase

class GUNSystem() : SubsystemBase() {

    private val sparkMaxElevator = CANSparkMax(11, CANSparkLowLevel.MotorType.kBrushless)

    fun goToPosition(angle: Double, elevatorHeight: Double) {

    }

    fun setSpeed(left: Double, right: Double) {
    }

    fun shoot(angle: Double, elevatorHeight: Double, leftPower: Double, rightPower: Double) {
        goToPosition(angle, elevatorHeight)
        setSpeed(leftPower, rightPower)
    }

    fun elevate(speed : Double) {
        sparkMaxElevator.set(speed)
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
