package frc.robot.subsystems

import com.pathplanner.lib.auto.AutoBuilder
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.constants.DriveConstants
import frc.robot.constants.PathPlannerLibConstants
import org.littletonrobotics.junction.Logger
import swervelib.SwerveDrive
import swervelib.telemetry.SwerveDriveTelemetry

class SwerveSystem(private val io: SwerveSystemIO) : SubsystemBase() {
    val inputs: SwerveSystemIO.SwerveSystemIOInputs = SwerveSystemIO.SwerveSystemIOInputs

    val swerveDrive = SwerveDrive(
        DriveConstants.DRIVE_CONFIG,
        DriveConstants.CONTRROLLER_CONFIG,
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

    fun isRed(): Boolean =
        DriverStation.getAlliance().isPresent && DriverStation.getAlliance().get() == DriverStation.Alliance.Red

    override fun periodic() {
        io.updateInputs(inputs)
    }
}