package frc.robot.subsystems

import com.pathplanner.lib.auto.AutoBuilder
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.constants.DriveConstants
import frc.robot.constants.PathPlannerLibConstants
import frc.robot.util.DualPigeon2Swerve
import swervelib.SwerveDrive
import swervelib.encoders.CANCoderSwerve
import swervelib.motors.SparkMaxSwerve
import swervelib.parser.*
import swervelib.parser.json.MotorConfigDouble
import swervelib.telemetry.SwerveDriveTelemetry

class SwerveSystem(private val io: SwerveSystemIO) : SubsystemBase() {
    private val inputs: SwerveSystemIO.SwerveSystemIOInputs = SwerveSystemIO.SwerveSystemIOInputs

    val swerveDrive: SwerveDrive

    init {
        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH

//        val kinematics = SwerveDriveKinematics(
//            Translation2d(
//                // Front Left
//                Units.inchesToMeters(DriveConstants.MODULE_X_OFFSET),
//                Units.inchesToMeters(DriveConstants.MODULE_Y_OFFSET),
//            ),
//            Translation2d(
//                // Front Right
//                Units.inchesToMeters(DriveConstants.MODULE_X_OFFSET),
//                Units.inchesToMeters(-DriveConstants.MODULE_Y_OFFSET),
//            ),
//            Translation2d(
//                // Back Left
//                Units.inchesToMeters(-DriveConstants.MODULE_X_OFFSET),
//                Units.inchesToMeters(DriveConstants.MODULE_Y_OFFSET),
//            ),
//            Translation2d(
//                // Back Right
//                Units.inchesToMeters(-DriveConstants.MODULE_X_OFFSET),
//                Units.inchesToMeters(-DriveConstants.MODULE_Y_OFFSET),
//            ),
//        )

        val conversionFactors = MotorConfigDouble(0.045777493, 16.8)

        val drivePID = PIDFConfig(0.00023, 0.0000002, 1.0)
        val twistPID = PIDFConfig(0.004, 1.5)

        val moduleCharacteristics = SwerveModulePhysicalCharacteristics(
            conversionFactors,
            DriveConstants.WHEEL_GRIP_COEFFICIENT_OF_FRICTION,
            DriveConstants.OPTIMAL_VOLTAGE,
            DriveConstants.DRIVE_MOTOR_CURRENT_LIMIT,
            DriveConstants.TWIST_MOTOR_CURRENT_LIMIT,
            DriveConstants.DRIVE_MOTOR_RAMP_RATE,
            DriveConstants.TWIST_MOTOR_RAMP_RATE,
        )

        val driveConfig = SwerveDriveConfiguration(
            arrayOf(
                SwerveModuleConfiguration(
                    SparkMaxSwerve(DriveConstants.FRONT_LEFT_DRIVE_ID, true),
                    SparkMaxSwerve(DriveConstants.FRONT_LEFT_TWIST_ID, false),
                    conversionFactors,
                    CANCoderSwerve(DriveConstants.FRONT_LEFT_ENCODER_ID),
                    DriveConstants.FRONT_LEFT_ANGLE_OFFSET,
                    DriveConstants.MODULE_X_OFFSET,
                    DriveConstants.MODULE_Y_OFFSET,
                    twistPID,
                    drivePID,
                    moduleCharacteristics,
                    "Front Left Swerve Module"
                ),
                SwerveModuleConfiguration(
                    SparkMaxSwerve(DriveConstants.FRONT_RIGHT_DRIVE_ID, true),
                    SparkMaxSwerve(DriveConstants.FRONT_RIGHT_TWIST_ID, false),
                    conversionFactors,
                    CANCoderSwerve(DriveConstants.FRONT_RIGHT_ENCODER_ID),
                    DriveConstants.FRONT_RIGHT_ANGLE_OFFSET,
                    DriveConstants.MODULE_X_OFFSET,
                    -DriveConstants.MODULE_Y_OFFSET,
                    twistPID,
                    drivePID,
                    moduleCharacteristics,
                    "Front Right Swerve Module"
                ),
                SwerveModuleConfiguration(
                    SparkMaxSwerve(DriveConstants.BACK_RIGHT_DRIVE_ID, true),
                    SparkMaxSwerve(DriveConstants.BACK_RIGHT_TWIST_ID, false),
                    conversionFactors,
                    CANCoderSwerve(DriveConstants.BACK_RIGHT_ENCODER_ID),
                    DriveConstants.BACK_RIGHT_ANGLE_OFFSET,
                    -DriveConstants.MODULE_X_OFFSET,
                    -DriveConstants.MODULE_Y_OFFSET,
                    twistPID,
                    drivePID,
                    moduleCharacteristics,
                    "Back Right Swerve Module"
                ),
                SwerveModuleConfiguration(
                    SparkMaxSwerve(DriveConstants.BACK_LEFT_DRIVE_ID, true),
                    SparkMaxSwerve(DriveConstants.BACK_LEFT_TWIST_ID, false),
                    conversionFactors,
                    CANCoderSwerve(DriveConstants.BACK_LEFT_ENCODER_ID),
                    DriveConstants.BACK_LEFT_ANGLE_OFFSET,
                    -DriveConstants.MODULE_X_OFFSET,
                    DriveConstants.MODULE_Y_OFFSET,
                    twistPID,
                    drivePID,
                    moduleCharacteristics,
                    "Back Left Swerve Module"
                )
            ),
            DualPigeon2Swerve(
                DriveConstants.NORMAL_PIGEON_ID,
                DriveConstants.REVERSE_PIGEON_ID,
                "",
                Rotation3d(), // idk what this should be
            ),
            false,
            SimpleMotorFeedforward(0.0, 0.0),
            moduleCharacteristics,
        )

        swerveDrive = SwerveDrive(
            driveConfig,
            SwerveControllerConfiguration(
                driveConfig,
                PIDFConfig(0.4, 0.01),
                DriveConstants.ANGLE_JOYSTICK_RADIUS_DEADBAND,
                DriveConstants.MAX_SPEED,
            ),
            DriveConstants.MAX_SPEED,
        )

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