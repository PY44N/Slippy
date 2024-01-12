package frc.robot.subsystems

import com.revrobotics.CANSparkMax
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.Filesystem
import frc.robot.constants.MotorConstants
import swervelib.SwerveDrive
import swervelib.motors.SparkMaxSwerve
import swervelib.motors.SwerveMotor
import swervelib.parser.SwerveDriveConfiguration
import swervelib.parser.SwerveModuleConfiguration
import swervelib.parser.SwerveParser
import java.io.File

object SwerveSystem {
    val frontLeft = SwerveModuleConfiguration(SparkMaxSwerve(CANSparkMax(MotorConstants)))

    val swerveDrive = SwerveDrive(SwerveDriveConfiguration())

    fun drive(translation: Translation2d, rotation: Double, fieldRelative: Boolean) {
        swerveDrive.drive(translation, rotation, fieldRelative, false)
    }
}