package frc.robot.subsystems.cannon

interface CannonIO {

    public fun setLeftShooter(vel: Double)
    public fun setRightShooter(vel: Double)

    public fun getLeftShooterVel(): Double
    public fun getRightShooterVel(): Double

    public fun setInnerIntakePercent(percent: Double)
    public fun setOuterIntakePercent(percent: Double)

    public fun getExitBeamBreak(): Boolean
    public fun getEntryBeamBreak(): Boolean
    public fun getLoadedBeamBreak(): Boolean
}
