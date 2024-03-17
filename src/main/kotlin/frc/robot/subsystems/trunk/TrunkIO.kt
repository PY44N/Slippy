package frc.robot.subsystems.trunk

interface TrunkIO {
    var positionBrake: Boolean
    var rotationBrake: Boolean
    fun getPosition(): Double
    fun getThroughBoreRawRotation(): Double
    fun getFalconRawRotation(): Double
    fun setFalconThroughBoreOffset()
    fun setElevatorSpeed(speed: Double)
    fun getPivotVelocity(): Double
    fun setZeroPosition()
    fun setRotationVoltage(volts: Double)
    fun atTopLimit(): Boolean
    fun setServoAngle(angle: Double)
    fun periodic()
}