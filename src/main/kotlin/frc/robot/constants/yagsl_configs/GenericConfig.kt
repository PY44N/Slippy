package frc.robot.constants.yagsl_configs

import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.util.Units
import swervelib.encoders.SwerveAbsoluteEncoder
import swervelib.imu.SwerveIMU
import swervelib.motors.SwerveMotor
import swervelib.parser.PIDFConfig
import swervelib.parser.SwerveControllerConfiguration
import swervelib.parser.SwerveDriveConfiguration
import swervelib.parser.SwerveModuleConfiguration
import swervelib.parser.SwerveModulePhysicalCharacteristics
import swervelib.parser.json.MotorConfigDouble

class YAGSLConfig(
    val imu: SwerveIMU,
    val invertedIMU: Boolean,
    val angleJoystickDeadband: Double,
    val maxSpeedMPS: Double,
    val moduleXOffset: Double,
    val moduleYOffset: Double,

    val conversionFactors: MotorConfigDouble,
    val wheelFriction: Double,
    val optimalVoltage: Double,
    val driveCurrentLimit: Int,
    val twistCurrentLimit: Int,
    val driveRampRate: Double,
    val twistRampRate: Double,

    val flDrive: SwerveMotor,
    val flDriveInverted: Boolean,
    val flTwist: SwerveMotor,
    val flTwistInverted: Boolean,
    val flEncoder: SwerveAbsoluteEncoder,
    val flOffset: Double,

    val frDrive: SwerveMotor,
    val frDriveInverted: Boolean,
    val frTwist: SwerveMotor,
    val frTwistInverted: Boolean,
    val frEncoder: SwerveAbsoluteEncoder,
    val frOffset: Double,

    val brDrive: SwerveMotor,
    val brDriveInverted: Boolean,
    val brTwist: SwerveMotor,
    val brTwistInverted: Boolean,
    val brEncoder: SwerveAbsoluteEncoder,
    val brOffset: Double,

    val blDrive: SwerveMotor,
    val blDriveInverted: Boolean,
    val blTwist: SwerveMotor,
    val blTwistInverted: Boolean,
    val blEncoder: SwerveAbsoluteEncoder,
    val blOffset: Double,

    val drivePID: PIDFConfig,
    val twistPID: PIDFConfig,
    val headingPID: PIDFConfig,
    val driveFeedForward: SimpleMotorFeedforward,

    ) {
    val kinematics = SwerveDriveKinematics(
        Translation2d(
            // Front Left
            Units.inchesToMeters(moduleXOffset),
            Units.inchesToMeters(moduleYOffset),
        ),
        Translation2d(
            // Front Right
            Units.inchesToMeters(moduleXOffset),
            Units.inchesToMeters(-moduleYOffset),
        ),
        Translation2d(
            // Back Right
            Units.inchesToMeters(-moduleXOffset),
            Units.inchesToMeters(-moduleYOffset),
        ),
        Translation2d(
            // Back Right
            Units.inchesToMeters(-moduleXOffset),
            Units.inchesToMeters(moduleYOffset),
        ),
    )
    val moduleCharacteristics = SwerveModulePhysicalCharacteristics(
        conversionFactors,
        wheelFriction,
        optimalVoltage,
        driveCurrentLimit,
        twistCurrentLimit,
        SlippyConfig.DRIVE_MOTOR_RAMP_RATE,
        SlippyConfig.TWIST_MOTOR_RAMP_RATE,
    )

    val flConfig = SwerveModuleConfiguration(
        flDrive,
        flTwist,
        conversionFactors,
        flEncoder,
        flOffset,
        moduleXOffset,
        moduleYOffset,
        twistPID,
        drivePID,
        moduleCharacteristics,
        "Front Left Swerve Module"
    )
    val frConfig = SwerveModuleConfiguration(
        frDrive,
        frTwist,
        conversionFactors,
        frEncoder,
        frOffset,
        moduleXOffset,
        -moduleYOffset,
        twistPID,
        drivePID,
        moduleCharacteristics,
        "Front Right Swerve Module"
    )
    val brConfig = SwerveModuleConfiguration(
        brDrive,
        brTwist,
        conversionFactors,
        brEncoder,
        brOffset,
        -moduleXOffset,
        -moduleYOffset,
        twistPID,
        drivePID,
        moduleCharacteristics,
        "Back Right Swerve Module"
    )
    val blConfig = SwerveModuleConfiguration(
        blDrive,
        blTwist,
        conversionFactors,
        blEncoder,
        blOffset,
        -moduleXOffset,
        moduleYOffset,
        twistPID,
        drivePID,
        moduleCharacteristics,
        "Back Left Swerve Module"
    )

    val driveConfig = SwerveDriveConfiguration(
        arrayOf(flConfig, frConfig, brConfig, blConfig),
        imu,
        false,
        driveFeedForward,
        moduleCharacteristics,
    )

    val controllerConfig = SwerveControllerConfiguration(
        driveConfig,
        headingPID,
        angleJoystickDeadband,
        maxSpeedMPS,
    )
}