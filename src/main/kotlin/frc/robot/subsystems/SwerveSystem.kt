package frc.robot.subsystems

import com.revrobotics.CANSparkMax
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.constants.MotorConstants
import swervelib.SwerveDrive
import swervelib.motors.SparkMaxSwerve
import swervelib.motors.SwerveMotor
import swervelib.parser.SwerveDriveConfiguration
import swervelib.parser.SwerveModuleConfiguration
import swervelib.parser.SwerveParser
import swervelib.telemetry.SwerveDriveTelemetry
import java.io.File

class SwerveSystem(directory: File) : SubsystemBase() {
    private val swerveDrive: SwerveDrive;

    init {
        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH
        try {
            swerveDrive = SwerveParser(directory).createSwerveDrive(10.0)
        } catch (e: Exception) {
            throw RuntimeException(e)
        }
        swerveDrive.setHeadingCorrection(false)
    }

    fun drive(translation: Translation2d, rotation: Double, fieldRelative: Boolean) {
        swerveDrive.drive(translation, rotation, fieldRelative, false)
    }
}