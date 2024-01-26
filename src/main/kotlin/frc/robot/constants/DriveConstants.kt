package frc.robot.constants

import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.util.Units
import frc.robot.util.DualPigeon2Swerve
import swervelib.encoders.CANCoderSwerve
import swervelib.motors.SparkMaxSwerve
import swervelib.parser.*
import swervelib.parser.json.MotorConfigDouble

object DriveConstants {
    // TODO: make these ids not bad (1-4 clockwise)
    const val FRONT_LEFT_DRIVE_ID: Int = 6
    const val FRONT_RIGHT_DRIVE_ID: Int = 8
    const val BACK_RIGHT_DRIVE_ID: Int = 7
    const val BACK_LEFT_DRIVE_ID: Int = 5

    const val FRONT_LEFT_TWIST_ID: Int = 10
    const val FRONT_RIGHT_TWIST_ID: Int = 12
    const val BACK_RIGHT_TWIST_ID: Int = 11
    const val BACK_LEFT_TWIST_ID: Int = 9

    const val FRONT_LEFT_ENCODER_ID: Int = 2
    const val FRONT_RIGHT_ENCODER_ID: Int = 4
    const val BACK_RIGHT_ENCODER_ID: Int = 3
    const val BACK_LEFT_ENCODER_ID: Int = 1

    const val MAX_SPEED = 4.7

    const val NORMAL_PIGEON_ID = 30
    const val REVERSE_PIGEON_ID = 31

    val IMU = DualPigeon2Swerve(
        NORMAL_PIGEON_ID,
        REVERSE_PIGEON_ID,
        "",
        Rotation3d(), // idk what this should be
    )

    // these are random
    // module positions (center of robot to center of wheel)
    const val MODULE_X_OFFSET = 12.375
    const val MODULE_Y_OFFSET = 12.375

    val KINEMATICS = SwerveDriveKinematics(
        Translation2d(
            // Front Left
            Units.inchesToMeters(MODULE_X_OFFSET),
            Units.inchesToMeters(MODULE_Y_OFFSET),
        ),
        Translation2d(
            // Front Right
            Units.inchesToMeters(MODULE_X_OFFSET),
            Units.inchesToMeters(-MODULE_Y_OFFSET),
        ),
        Translation2d(
            // Back Left
            Units.inchesToMeters(-MODULE_X_OFFSET),
            Units.inchesToMeters(MODULE_Y_OFFSET),
        ),
        Translation2d(
            // Back Right
            Units.inchesToMeters(-MODULE_X_OFFSET),
            Units.inchesToMeters(-MODULE_Y_OFFSET),
        ),
    )


    // idk if these are right
    const val WHEEL_GRIP_COEFFICIENT_OF_FRICTION = 1.19
    const val OPTIMAL_VOLTAGE = 12.0
    const val DRIVE_MOTOR_CURRENT_LIMIT = 40
    const val TWIST_MOTOR_CURRENT_LIMIT = 20
    const val DRIVE_MOTOR_RAMP_RATE = 0.25
    const val TWIST_MOTOR_RAMP_RATE = 0.25

    const val ANGLE_JOYSTICK_RADIUS_DEADBAND = 0.5

    // TODO: make parametric
    val CONVERSION_FACTORS = MotorConfigDouble(0.045777493, 16.8)

    val DRIVE_PID = PIDFConfig(0.00023, 0.0000002, 1.0)
    val TWIST_PID = PIDFConfig(0.004, 1.5)
    val HEADING_PID = PIDFConfig(.4, .01)

    val FRONT_LEFT_DRIVE_MOTOR = SparkMaxSwerve(FRONT_LEFT_DRIVE_ID, true)
    val FRONT_LEFT_TWIST_MOTOR = SparkMaxSwerve(FRONT_LEFT_TWIST_ID, false)
    val FRONT_LEFT_ENCODER = CANCoderSwerve(FRONT_LEFT_ENCODER_ID)
    const val FRONT_LEFT_ANGLE_OFFSET = -114.609

    val FRONT_RIGHT_DRIVE_MOTOR = SparkMaxSwerve(FRONT_RIGHT_DRIVE_ID, true)
    val FRONT_RIGHT_TWIST_MOTOR = SparkMaxSwerve(FRONT_RIGHT_TWIST_ID, false)
    val FRONT_RIGHT_ENCODER = CANCoderSwerve(FRONT_RIGHT_ENCODER_ID)
    const val FRONT_RIGHT_ANGLE_OFFSET = -50.977

    val BACK_RIGHT_DRIVE_MOTOR = SparkMaxSwerve(BACK_RIGHT_DRIVE_ID, true)
    val BACK_RIGHT_TWIST_MOTOR = SparkMaxSwerve(BACK_RIGHT_TWIST_ID, false)
    val BACK_RIGHT_ENCODER = CANCoderSwerve(BACK_RIGHT_ENCODER_ID)
    const val BACK_RIGHT_ANGLE_OFFSET = -18.281

    val BACK_LEFT_DRIVE_MOTOR = SparkMaxSwerve(BACK_LEFT_DRIVE_ID, true)
    val BACK_LEFT_TWIST_MOTOR = SparkMaxSwerve(BACK_LEFT_TWIST_ID, false)
    val BACK_LEFT_ENCODER = CANCoderSwerve(BACK_LEFT_ENCODER_ID)
    const val BACK_LEFT_ANGLE_OFFSET = 6.504

    val MODULE_CHARACTERISTICS = SwerveModulePhysicalCharacteristics(
        CONVERSION_FACTORS,
        WHEEL_GRIP_COEFFICIENT_OF_FRICTION,
        OPTIMAL_VOLTAGE,
        DRIVE_MOTOR_CURRENT_LIMIT,
        TWIST_MOTOR_CURRENT_LIMIT,
        DRIVE_MOTOR_RAMP_RATE,
        TWIST_MOTOR_RAMP_RATE,
    )

    // not sure that the signs are right
    val FRONT_LEFT_MODULE_CONFIG = SwerveModuleConfiguration(
        FRONT_LEFT_DRIVE_MOTOR,
        FRONT_LEFT_TWIST_MOTOR,
        CONVERSION_FACTORS,
        FRONT_LEFT_ENCODER,
        FRONT_LEFT_ANGLE_OFFSET,
        MODULE_X_OFFSET,
        MODULE_Y_OFFSET,
        TWIST_PID,
        DRIVE_PID,
        MODULE_CHARACTERISTICS,
        "Front Left Swerve Module"
    )

    val FRONT_RIGHT_MODULE_CONFIG = SwerveModuleConfiguration(
        FRONT_RIGHT_DRIVE_MOTOR,
        FRONT_RIGHT_TWIST_MOTOR,
        CONVERSION_FACTORS,
        FRONT_RIGHT_ENCODER,
        FRONT_RIGHT_ANGLE_OFFSET,
        MODULE_X_OFFSET,
        -MODULE_Y_OFFSET,
        TWIST_PID,
        DRIVE_PID,
        MODULE_CHARACTERISTICS,
        "Front Right Swerve Module"
    )

    val BACK_RIGHT_MODULE_CONFIG = SwerveModuleConfiguration(
        BACK_RIGHT_DRIVE_MOTOR,
        BACK_RIGHT_TWIST_MOTOR,
        CONVERSION_FACTORS,
        BACK_RIGHT_ENCODER,
        BACK_RIGHT_ANGLE_OFFSET,
        -MODULE_X_OFFSET,
        -MODULE_Y_OFFSET,
        TWIST_PID,
        DRIVE_PID,
        MODULE_CHARACTERISTICS,
        "Back Right Swerve Module"
    )

    val BACK_LEFT_MODULE_CONFIG = SwerveModuleConfiguration(
        BACK_LEFT_DRIVE_MOTOR,
        BACK_LEFT_TWIST_MOTOR,
        CONVERSION_FACTORS,
        BACK_LEFT_ENCODER,
        BACK_LEFT_ANGLE_OFFSET,
        -MODULE_X_OFFSET,
        MODULE_Y_OFFSET,
        TWIST_PID,
        DRIVE_PID,
        MODULE_CHARACTERISTICS,
        "Back Left Swerve Module"
    )

    // random numbers
    val DRIVE_FEED_FORWARD = SimpleMotorFeedforward(0.0, 0.0)

    val DRIVE_CONFIG = SwerveDriveConfiguration(
        arrayOf(FRONT_LEFT_MODULE_CONFIG, FRONT_RIGHT_MODULE_CONFIG, BACK_RIGHT_MODULE_CONFIG, BACK_LEFT_MODULE_CONFIG),
        IMU,
        false,
        DRIVE_FEED_FORWARD,
        MODULE_CHARACTERISTICS,
    )

    val CONTROLLER_CONFIG = SwerveControllerConfiguration(
        DRIVE_CONFIG,
        HEADING_PID,
        ANGLE_JOYSTICK_RADIUS_DEADBAND,
        MAX_SPEED,
    )
}