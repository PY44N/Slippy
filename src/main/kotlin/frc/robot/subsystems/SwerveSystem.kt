package frc.robot.subsystems

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.constants.DriveConstants
import frc.robot.constants.PathPlannerLibConstants
import swervelib.SwerveDrive
import swervelib.telemetry.SwerveDriveTelemetry


class SwerveSystem() : SubsystemBase() {

    val gyro = DriveConstants.imu

    val swerveDrive =  SwerveDrive(
        DriveConstants.driveConfig,
        DriveConstants.controllerConfig,
        DriveConstants.MAX_SPEED,
    )

    init {
        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH

        swerveDrive.setHeadingCorrection(false)
        swerveDrive.setMotorIdleMode(false)
        swerveDrive.pushOffsetsToControllers()

        setupPathPlanner()
    }

    fun setupPathPlanner() {
        AutoBuilder.configureHolonomic(
            swerveDrive::getPose,
            swerveDrive::resetOdometry,
            swerveDrive::getRobotVelocity,
            this::autoDrive,
            PathPlannerLibConstants.pathPlannerConfig,
            this::isRed,
            this,
        )
    }

    fun drive(translation: Translation2d, rotation: Double, fieldRelative: Boolean) {
        swerveDrive.drive(translation, rotation, fieldRelative, false)
    }

    fun autoDrive(velocity: ChassisSpeeds) {
        swerveDrive.drive(velocity)
    }

    fun isRed() : Boolean { // default blue
        var alliance = DriverStation.getAlliance()
        if (alliance.isPresent())
            return alliance.get() == DriverStation.Alliance.Red
        return false
    }

    fun getAutonomousCommand(pathName: String?, setOdomToStart: Boolean): Command {
        // Load the path you want to follow using its name in the GUI
        val path = PathPlannerPath.fromPathFile(pathName)

        if (setOdomToStart) {
            resetOdometry(Pose2d(path.getPoint(0).position, getHeading()))
        }

        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(path)
    }
}