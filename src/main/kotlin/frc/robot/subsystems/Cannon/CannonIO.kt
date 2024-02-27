package frc.robot.subsystems.Cannon

interface CannonIO {

    fun setLeftVel(speed: Double)
    fun setRightVel(speed: Double)


    fun getExitBeamBreak(): Boolean

    fun getLeftVel(): Double
    fun getRightVel(): Double
}
