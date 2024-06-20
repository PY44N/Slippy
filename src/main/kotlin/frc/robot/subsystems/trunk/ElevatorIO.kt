package frc.robot.subsystems.trunk

interface ElevatorIO {
    fun setDesiredPosition(position: Double)
    fun getPosition(): Double

    fun update()
}