package frc.robot.util

import edu.wpi.first.networktables.NetworkTableInstance



data class Shot(
    val shooterAngle: Double,
    val elevatorHeight: Double,
    val leftPower: Double,
    val rightPower: Double,
) {
    fun toCSV() : String {
        return "${shooterAngle}, ${elevatorHeight}, ${leftPower}, ${rightPower}"
    }
}