package frc.robot.subsystems

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.Filesystem
import swervelib.parser.SwerveParser
import java.io.File

object SwerveSystem {
    val swerveDrive = SwerveParser(File(Filesystem.getDeployDirectory(), "swerve")).createSwerveDrive(10.0, 1.0, 1.0 /* TODO: They are NOT geared 1:1 */)

    fun drive(translation: Translation2d, rotation: Double, fieldRelative: Boolean) {
        swerveDrive.drive(translation, rotation, fieldRelative, false)
    }
}