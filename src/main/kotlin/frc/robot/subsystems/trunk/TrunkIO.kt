package frc.robot.subsystems.trunk

interface TrunkIO {
    // TODO: I have no wifi, so I can't see how to actually advantagekit this
    var positionBrake: Boolean
    var rotationBrake: Boolean
    fun getRawPosition(): Double
    fun getThroughBoreRawRotation(): Double
    fun getFalconRawRotation(): Double
    fun setFalconThroughBoreOffset()
    fun setElevatorSpeed(speed: Double)
    fun setZeroPosition()
    fun setRotationVoltage(volts: Double)
    fun atTopLimit(): Boolean
    fun periodic()
}