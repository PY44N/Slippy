package frc.robot.constants

import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.util.Units
import frc.robot.util.DualPigeon2Swerve
import swervelib.SwerveController
import swervelib.SwerveModule
import swervelib.encoders.CANCoderSwerve
import swervelib.motors.SparkMaxSwerve
import swervelib.parser.PIDFConfig
import swervelib.parser.SwerveControllerConfiguration
import swervelib.parser.SwerveDriveConfiguration
import swervelib.parser.SwerveModuleConfiguration
import swervelib.parser.SwerveModulePhysicalCharacteristics
import swervelib.parser.json.MotorConfigDouble

object DriveConstants {
    // TODO: make these ids not bad (1-4 clockwise)
    const val FRONT_LEFT_DRIVE: Int = 6
    const val FRONT_RIGHT_DRIVE: Int = 8
    const val BACK_RIGHT_DRIVE: Int = 7
    const val BACK_LEFT_DRIVE: Int = 5

    const val FRONT_LEFT_TWIST: Int = 10
    const val FRONT_RIGHT_TWIST: Int = 12
    const val BACK_RIGHT_TWIST: Int = 11
    const val BACK_LEFT_TWIST: Int = 9

    const val FRONT_LEFT_ENCODER: Int = 2
    const val FRONT_RIGHT_ENCODER: Int = 4
    const val BACK_RIGHT_ENCODER: Int = 3
    const val BACK_LEFT_ENCODER: Int = 1

    const val MAX_SPEED = 14.5 // please find the real value

    const val NORMAL_PIGEON = 30
    const val REVERSE_PIGEON = 31

    val imu = DualPigeon2Swerve(
        NORMAL_PIGEON,
        REVERSE_PIGEON,
        "",
        Rotation3d(), // idk what this should be
    )

    // these are random
    // module positions (center of robot to center of wheel)
    const val MODULE_X_OFFSET = 12.5
    const val MODULE_Y_OFFSET = 12.5

    val kinematics = SwerveDriveKinematics(
        Translation2d( // Front Left
            Units.inchesToMeters(MODULE_X_OFFSET),
            Units.inchesToMeters(MODULE_Y_OFFSET),
        ),
        Translation2d( // Front Right
            Units.inchesToMeters(MODULE_X_OFFSET),
            Units.inchesToMeters(-MODULE_Y_OFFSET),
        ),
        Translation2d( // Back Left
            Units.inchesToMeters(-MODULE_X_OFFSET),
            Units.inchesToMeters(MODULE_Y_OFFSET),
        ),
        Translation2d( // Back Right
            Units.inchesToMeters(-MODULE_X_OFFSET),
            Units.inchesToMeters(-MODULE_Y_OFFSET),
        ),
    )


    // idk if these are right
    const val wheelGripCoefficientOfFriction = 1.19
    const val optimalVoltage = 12.0
    const val driveMotorCurrentLimit = 40
    const val twistMotorCurrentLimit = 20
    const val driveMotorRampRate = 0.25
    const val twistMotorRampRate = 0.25

    const val angleJoyStickRadiusDeadband = 0.5
    // TODO: make parametric
    val conversionFactors = MotorConfigDouble(0.045777493, 16.8)

    val drivePID = PIDFConfig(0.00023, 0.0000002, 1.0)
    val twistPID = PIDFConfig(0.004, 1.5)
    val headingPID = PIDFConfig(.4, .01)

    val flDrive = SparkMaxSwerve(FRONT_LEFT_DRIVE, true)
    val flTwist = SparkMaxSwerve(FRONT_LEFT_TWIST, false)
    val flEncoder = CANCoderSwerve(FRONT_LEFT_ENCODER)
    const val flAngleOffset = -114.609
    val frDrive = SparkMaxSwerve(FRONT_RIGHT_DRIVE, true)
    val frTwist = SparkMaxSwerve(FRONT_RIGHT_TWIST, false)
    val frEncoder = CANCoderSwerve(FRONT_RIGHT_ENCODER)
    const val frAngleOffset = -50.977
    val brDrive = SparkMaxSwerve(BACK_RIGHT_DRIVE, true)
    val brTwist = SparkMaxSwerve(BACK_RIGHT_TWIST, false)
    val brEncoder = CANCoderSwerve(BACK_RIGHT_ENCODER)
    const val brAngleOffset = -18.281
    val blDrive = SparkMaxSwerve(BACK_LEFT_DRIVE, true)
    val blTwist = SparkMaxSwerve(BACK_LEFT_TWIST, false)
    val blEncoder = CANCoderSwerve(BACK_LEFT_ENCODER)
    const val blAngleOffset = 6.504

    val moduleCharacteristics = SwerveModulePhysicalCharacteristics(
        conversionFactors,
        wheelGripCoefficientOfFriction,
        optimalVoltage,
        driveMotorCurrentLimit,
        twistMotorCurrentLimit,
        driveMotorRampRate,
        twistMotorRampRate,
    )

    // not sure that the signs are right
    val flModuleConfig = SwerveModuleConfiguration(
        flDrive,
        flTwist,
        conversionFactors,
        flEncoder,
        flAngleOffset,
        MODULE_X_OFFSET,
        MODULE_Y_OFFSET,
        twistPID,
        drivePID,
        moduleCharacteristics,
        "Front Left Swerve Module"
    )

    val frModuleConfig = SwerveModuleConfiguration(
        frDrive,
        frTwist,
        conversionFactors,
        frEncoder,
        frAngleOffset,
        MODULE_X_OFFSET,
        -MODULE_Y_OFFSET,
        twistPID,
        drivePID,
        moduleCharacteristics,
        "Front Right Swerve Module"
    )

    val brModuleConfig = SwerveModuleConfiguration(
        brDrive,
        brTwist,
        conversionFactors,
        brEncoder,
        brAngleOffset,
        -MODULE_X_OFFSET,
        -MODULE_Y_OFFSET,
        twistPID,
        drivePID,
        moduleCharacteristics,
        "Back Right Swerve Module"
    )

    val blModuleConfig = SwerveModuleConfiguration(
        blDrive,
        blTwist,
        conversionFactors,
        blEncoder,
        blAngleOffset,
        -MODULE_X_OFFSET,
        MODULE_Y_OFFSET,
        twistPID,
        drivePID,
        moduleCharacteristics,
        "Back Left Swerve Module"
    )
    // random numbers
    val driveFeedForward = SimpleMotorFeedforward(0.0,0.0)

    val driveConfig = SwerveDriveConfiguration(
        arrayOf(flModuleConfig, frModuleConfig, brModuleConfig, blModuleConfig),
        imu,
        false,
        driveFeedForward,
        moduleCharacteristics,
    )

    val controllerConfig = SwerveControllerConfiguration(
        driveConfig,
        headingPID,
        angleJoyStickRadiusDeadband,
        MAX_SPEED,
    )
}