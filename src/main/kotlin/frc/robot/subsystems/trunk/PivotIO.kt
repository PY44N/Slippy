package frc.robot.subsystems.trunk

interface PivotIO {
    fun setDesiredRotation(rotationDegrees: Double)
    fun getRotation(): Double
    fun setCoast(coast: Boolean)

    fun update()
}