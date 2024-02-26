package frc.robot.subsystems.trunk

interface TrunkIO {
    // TODO: I have no wifi, so I can't see how to actually advantagekit this
    fun getPosition(): Double
    fun getRotation(): Double

    fun setDesiredPosition(position: Double)
    fun setDesiredRotation(angle: Double)
    fun setElevatorSpeed(speed: Double)
    fun setRotationSpeed(speed: Double)
    fun setZeroPosition()
    fun setZeroRotation()
    fun atTopLimit(): Boolean
    fun calibrate()

    fun periodic()
}