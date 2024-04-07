package frc.robot.subsystems.trunk

interface TrunkIO {
    fun directlySetPercentElevatorFollower(percent: Double)

    fun directlySetPercentElevatorMaster(percent: Double)
    var positionBrake: Boolean
    var rotationBrake: Boolean
    fun getPosition(): Double
    fun getThroughBoreRawRotation(): Double
    fun getFalconRawRotation(): Double
    fun setFalconThroughBoreOffset()
    fun setElevatorSpeed(speed: Double)
    fun getPivotVelocity(): Double
    fun setZeroPosition(top: Boolean)
    fun setRotationVoltage(volts: Double)
    fun getElevatorVelocity(): Double
    fun getElevatorMotorAccel(): Double
    fun atStowLimit(): Boolean
    fun atTopLimit(): Boolean
    fun setServoAngle(angle: Double)
    fun getServoAngle(): Double
    fun periodic()
}