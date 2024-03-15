package frc.robot.subsystems.trunk

import com.revrobotics.CANSparkBase

interface TrunkIO {
    var positionBrake: Boolean
    var rotationBrake: Boolean
    fun getRawPosition(): Double
    fun getRawRotation(): Double
    fun setElevatorSpeed(speed: Double)
    fun setZeroPosition()
    fun setRotationVoltage(volts: Double)
    fun atTopLimit(): Boolean
    fun setServoAngle(angle: Double)
    fun periodic()
}