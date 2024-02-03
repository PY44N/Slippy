package frc.robot.util

import java.io.File
import java.io.FileInputStream
import java.io.FileOutputStream
import java.io.InputStream
import java.io.OutputStream
import kotlin.io.bufferedWriter

class ShooterCalibrator(val directory: String) {
    val lastShot: Shot? = null
    fun readCsv(): List<Shot> {
        val inputStream = FileInputStream(directory)
        val reader = inputStream.bufferedReader()
        val header = reader.readLine()
        return reader.lineSequence()
            .filter { it.isNotBlank() }
            .map {
                val (angle, height, leftPower, rightPower) = it.split(',', ignoreCase = false, limit = 4)
                Shot(
                    angle.trim().toDouble(),
                    height.trim().toDouble(),
                    leftPower.trim().toDouble(),
                    rightPower.trim().toDouble(),
                    )
            }.toList()

    }
    fun OutputStream.writeCsv(shots: List<Shot>) {
        val writer = bufferedWriter()
        if(!File(directory).exists()) {
            writer.write(""""Shooter Angle", "Elevator Height", "Left Power", "Right Power"""")
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
}