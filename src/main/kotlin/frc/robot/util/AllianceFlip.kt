package frc.robot.util

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.DriverStation

object AllianceFlip {
    val fieldLength = Units.inchesToMeters(651.223)

    private fun apply(xPos: Double) =
        if (DriverStation.getAlliance().isPresent && DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
            fieldLength - xPos else xPos
    fun apply(pose: Pose2d) = Pose2d(apply(pose.x), pose.y, pose.rotation)
}