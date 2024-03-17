package frc.robot.util

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.DriverStation

object AllianceFlip {
    val fieldLength = Units.inchesToMeters(651.223)

    fun apply(xPos: Double): Double {
        return if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) fieldLength - xPos else xPos
    }

    fun apply(pos: Translation2d): Translation2d {
        return Translation2d(apply(pos.x), pos.y)
    }
}