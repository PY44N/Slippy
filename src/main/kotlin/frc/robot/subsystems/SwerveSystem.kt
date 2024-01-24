package frc.robot.subsystems

import com.pathplanner.lib.auto.AutoBuilder
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.constants.DriveConstants
import frc.robot.constants.PathPlannerLibConstants
import frc.robot.constants.YAGSLConstants
import swervelib.SwerveDrive
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.Filesystem
import frc.robot.util.DualPigeon2Swerve
import swervelib.parser.SwerveParser
import swervelib.telemetry.SwerveDriveTelemetry
import java.io.File

class SwerveSystem(imu: DualPigeon2Swerve) : SubsystemBase() {

    val swerveDrive: SwerveDrive

    init {


        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH

        swerveDrive.setHeadingCorrection(false)
        swerveDrive.setMotorIdleMode(false)
        swerveDrive.pushOffsetsToControllers()

        AutoBuilder.configureHolonomic(
            swerveDrive::getPose,
            swerveDrive::resetOdometry,
            swerveDrive::getRobotVelocity,
            this::autoDrive,
            PathPlannerLibConstants.pathPlannerConfig,
            this::getAlliance,
            this,
        )
    }

    fun drive(translation: Translation2d, rotation: Double, fieldRelative: Boolean) {
        swerveDrive.drive(translation, rotation, fieldRelative, false)
    }

    fun autoDrive(velocity: ChassisSpeeds) {
        swerveDrive.drive(velocity)
    }

    fun getAlliance() : Boolean {
        var alliance = DriverStation.getAlliance()
        if (alliance.isPresent())
            return alliance.get() == DriverStation.Alliance.Red
        return false
    }
}