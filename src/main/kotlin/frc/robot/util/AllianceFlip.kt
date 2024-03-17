package frc.robot.util

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.DriverStation

object AllianceFlip {
    private val fieldLength = Units.inchesToMeters(651.223)

    private fun apply(xPos: Double) =
        if (DriverStation.getAlliance() == null)
            xPos
        else if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
            fieldLength - xPos
        else
            xPos

    fun apply(pos: Pose2d) =
        Pose2d(apply(pos.x), pos.y, pos.rotation)
}