package frc.robot.util


data class Shot(
    val speakerDistance: Double,
    val parallelVelocity: Double,
    val perpendicularVelocity: Double,
    val robotSpeakerRelativeAngle: Double,
    val shooterAngle: Double,
    val elevatorHeight: Double,
    val leftPower: Double,
    val rightPower: Double,
) {
    fun toCSV(): String {
        return "${speakerDistance}, ${parallelVelocity}, ${perpendicularVelocity}, ${robotSpeakerRelativeAngle}, ${shooterAngle}, ${elevatorHeight}, ${leftPower}, ${rightPower}"
    }
}