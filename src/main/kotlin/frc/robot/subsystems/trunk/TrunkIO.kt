package frc.robot.subsystems.trunk

interface TrunkIO {
    // TODO: I have no wifi, so I can't see how to actually advantagekit this
    fun getPosition(): Double
    fun getRawRotation(): Double
    fun getRotation(): Double
    fun setDesiredPosition(position: Double)
    fun setDesiredRotation(angle: Double)
    fun setElevatorSpeed(speed: Double)
    fun setRotationSpeed(speed: Double)
    fun setZeroPosition(top: Boolean)
    fun setTopPositionLimit(position: Double)
    fun setBottomPositionLimit(position: Double)
    fun setTopRotationLimit(angle: Double)
    fun setBottomRotationLimit(angle: Double)
    fun atTopLimit(): Boolean
    fun atBottomLimit(): Boolean
    fun setPositionLimits(on: Boolean)
    fun setPID(on: Boolean)
    fun periodic()
}