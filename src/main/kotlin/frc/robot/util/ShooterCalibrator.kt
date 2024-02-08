package frc.robot.util

import edu.wpi.first.math.Vector
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Angle
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
            .map { line ->
                val list = line.split(",", ignoreCase = false, limit = 9).map{it.trim().toDouble()}
                Shot(
                    list[0],
                    list[1],
                    list[2],
                    list[3],
                    list[4],
                    list[5],
                    list[6],
                    list[7],
                    list[8],
                    )
            }.toList()
    }

    fun OutputStream.writeCsv(shots: List<Shot>) {
        val writer = bufferedWriter()
        if (!File(directory).exists()) {
            writer.write(""""x", "y", "vx", "vy", "robotAngle", "shooterAngle", "elevatorHeight", "leftPower", "rightPower"""")
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

    fun shoot(shooterAngle: Double, elevatorHeight: Double, leftPower: Double, rightPower: Double) {
        gun.shoot(shooterAngle, elevatorHeight, leftPower, rightPower)
        val botPose = RobotContainer.swerveSystem.swerveDrive.pose
        val v = RobotContainer.swerveSystem.swerveDrive.fieldVelocity
        lastShot = Shot(
            botPose.x,
            botPose.y,
            v.vxMetersPerSecond,
            v.vyMetersPerSecond,
            botPose.rotation.radians,
            shooterAngle,
            elevatorHeight,
            leftPower,
            rightPower,
        )
    }
    fun addLastShot() {

    }
}