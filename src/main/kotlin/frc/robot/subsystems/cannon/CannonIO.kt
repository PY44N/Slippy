package frc.robot.subsystems.cannon

interface CannonIO {
    fun getEntryBeamBreak(): Boolean
    fun getLoadedBeamBreak(): Boolean


    fun setShooterVel(leftRPM: Double, rightRPM: Double)

    fun setInnerIntakePercent(percent: Double)
    fun setOuterIntakePercent(percent: Double)

    fun periodic()
}