package frc.robot.util

import frc.robot.constants.FieldConstants
import kotlin.math.atan
import kotlin.math.pow
import kotlin.math.sqrt

data class Shot(
    val x: Double,
    val y: Double,
    val vx: Double,
    val vy: Double,
    val robotAngle: Double,
    val shooterAngle: Double,
    val leftPower: Double,
    val rightPower: Double,
) {
    fun toCSV(): String {
        return "${x}, ${y}, ${vx}, ${vy}, ${robotAngle}, ${shooterAngle}, ${leftPower}, ${rightPower}"
    }
}

data class PolarShot(
    val r: Double,
    val robotSpeakerRelativeAngle: Double,
    val leftPower: Double,
    val rightPower: Double,
    val shooterAngle: Double,
    val vInward: Double,
    val vTangent: Double,
) {
    fun toPolar(shot: Shot) : PolarShot {
        val d = arrayOf(FieldConstants.SPEAKER_CENTER_X - shot.x, FieldConstants.SPEAKER_CENTER_Y - shot.y)
        val speakerDistance = sqrt(d[0].pow(2) + d[1].pow(2))
        val inverseDistance = 1.0/speakerDistance
        val perpendicularVelocity = inverseDistance * (d[0] * shot.vx + d[1] * shot.vy)
        val parallelVelocity = inverseDistance * (d[0]*shot.vx - d[1] * shot.vy)
        val angle = atan(d[1]/d[0]) - shot.robotAngle
        return PolarShot(
            speakerDistance,
            angle,
            shot.leftPower,
            shot.rightPower,
            shot.shooterAngle,
            perpendicularVelocity,
            parallelVelocity,
        )
    }
}