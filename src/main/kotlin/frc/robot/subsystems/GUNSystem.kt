package frc.robot.subsystems

import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.wpilibj2.command.SubsystemBase

class GUNSystem() : SubsystemBase() {

    private val elevatorMotor = CANSparkMax(11, CANSparkLowLevel.MotorType.kBrushless)
    private val leftPivotMotor = CANSparkMax(13, CANSparkLowLevel.MotorType.kBrushless)
    private val rightPivotMotor = CANSparkMax(14, CANSparkLowLevel.MotorType.kBrushless)
//    private var currentPosition: Double

    init {

    }
    fun setZeroPosition() {

    }
    fun goToPosition(angle: Double, elevatorHeight: Double) {

    }

    fun setSpeed(left: Double, right: Double) {
    }

    fun shoot(angle: Double, elevatorHeight: Double, leftPower: Double, rightPower: Double) {
        goToPosition(angle, elevatorHeight)
        setSpeed(leftPower, rightPower)
    }

    fun elevate(speed : Double) {
        elevatorMotor.set(speed)
    }

    override fun periodic() {
    }

    override fun simulationPeriodic() {
    }

    companion object {
        const val MAX_PIVOT_HEIGHT_M = 0.64446
        const val MIN_PIVOT_HEIGHT_M = 0.348701
        const val THING_LENGTH_M = 0.6133
    }
}

enum class GUNPosition {
    AMP,
    SPEAKER,
    INTAKE,
    MOVIN,
}