package frc.robot.subsystems.GUN

interface GUNIO {

    fun setLeftSpeed(speed: Double)
    fun setRightSpeed(speed: Double)


    fun getExitBeamBreak(): Boolean

    fun getLeftSpeed(): Double
    fun getRightSpeed(): Double
}