package frc.robot.subsystems

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.util.HolonomicPathFollowerConfig
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.constants.DriveConstants
import frc.robot.constants.PathPlannerLibConstants
import org.littletonrobotics.junction.Logger
import swervelib.SwerveDrive
import swervelib.parser.SwerveParser
import swervelib.telemetry.SwerveDriveTelemetry
import java.io.File

class SwerveSystem(private val io: SwerveSystemIO, directory: File) : SubsystemBase() {
    private val inputs: SwerveSystemIO.SwerveSystemIOInputs = SwerveSystemIO.SwerveSystemIOInputs

    val swerveDrive: SwerveDrive

    init {
        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH

        try {
            swerveDrive = SwerveParser(directory).createSwerveDrive(DriveConstants.MAX_SPEED)
        } catch (e: Exception) {
            throw RuntimeException(e)
        }

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
            HolonomicPathFollowerConfig(
                PathPlannerLibConstants.translationPID,
                PathPlannerLibConstants.rotationPID,
                DriveConstants.MAX_SPEED,
                DriveConstants.ROBOT_RADIUS,
                PathPlannerLibConstants.replanningConfig,
            ),
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

    fun isRed(): Boolean =
        DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.recordOutput("RobotAccel", swerveDrive.accel.orElse(Translation3d(0.0, 0.0, 0.0)))
        Logger.recordOutput("RobotVelocity", swerveDrive.fieldVelocity)
        Logger.recordOutput("RobotRotation", swerveDrive.gyroRotation3d.angle)
        Logger.recordOutput("RobotPose", swerveDrive.pose)
    }
}
