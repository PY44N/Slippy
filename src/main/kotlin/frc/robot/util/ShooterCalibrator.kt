package frc.robot.util

import edu.wpi.first.math.Vector
import edu.wpi.first.math.geometry.Rotation2d
import frc.robot.RobotContainer
import frc.robot.constants.FieldConstants
import frc.robot.subsystems.GUNSystem
import java.io.File
import java.io.FileInputStream
import java.io.FileOutputStream
import java.io.OutputStream
import kotlin.math.sqrt
import kotlin.math.pow

class ShooterCalibrator(val directory: String, val gun: GUNSystem) {
    var lastShot: Shot? = null
    fun readCsv(): List<Shot> {
        val inputStream = FileInputStream(directory)
        val reader = inputStream.bufferedReader()
        val header = reader.readLine()
        return reader.lineSequence()
            .filter { it.isNotBlank() }
            .map {
                val list = it.split(",", ignoreCase = false, limit = 8)
                Shot(
                    list[0].trim().toDouble(),
                    list[1].trim().toDouble(),
                    list[2].trim().toDouble(),
                    list[3].trim().toDouble(),
                    list[4].trim().toDouble(),
                    list[5].trim().toDouble(),
                    list[6].trim().toDouble(),
                    list[7].trim().toDouble(),
                    )
            }.toList()
    }

    fun OutputStream.writeCsv(shots: List<Shot>) {
        val writer = bufferedWriter()
        if (!File(directory).exists()) {
            writer.write(""""speakerDistance", "parallelVelocity", "perpendicularVelocity", "robotSpeakerRelativeAngle", "shooterAngle", "elevatorHeight", "leftPower", "rightPower"""")
            writer.newLine()
        }
        shots.forEach {
            writer.write(it.toCSV())
            writer.newLine()
        }
        writer.flush()
    }

    fun writeOut(shots: List<Shot>) {
        FileOutputStream(directory).apply {
            writeCsv(shots)
        }
    }

    fun shoot(angle: Rotation2d, elevatorHeight: Double, leftPower: Double, rightPower: Double) {
        gun.shoot(angle, elevatorHeight, leftPower, rightPower)
        val botPose = RobotContainer.swerveSystem.swerveDrive.pose
        val v = RobotContainer.swerveSystem.swerveDrive.fieldVelocity
        val d = arrayOf(FieldConstants.SPEAKER_CENTER_X - botPose.x, FieldConstants.SPEAKER_CENTER_Y - botPose.y)
        val speakerDistance = sqrt(d[0].pow(2) + d[1].pow(2))
        val inverseDistance = 1.0/speakerDistance
        val parallelVelocity = inverseDistance * (d[0] * v.vxMetersPerSecond + d[1] * v.vyMetersPerSecond)
        val speakerRobotAngle
        lastShot = Shot(
            speakerDistance,
            parallelVelocity,
            ,
            ,

        )
    }
}