package frc.robot.subsystems

import com.pathplanner.lib.auto.AutoBuilder
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.constants.DriveConstants
import frc.robot.constants.PathPlannerLibConstants
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
        swerveDrive.modules[0].driveMotor.setInverted(false)
        swerveDrive.modules[0].angleMotor.setInverted(true)
        swerveDrive.modules[1].driveMotor.setInverted(false)
        swerveDrive.modules[1].angleMotor.setInverted(true)
        swerveDrive.modules[2].driveMotor.setInverted(true)
        swerveDrive.modules[2].angleMotor.setInverted(true)
        swerveDrive.modules[3].driveMotor.setInverted(true)
        swerveDrive.modules[3].angleMotor.setInverted(true)
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