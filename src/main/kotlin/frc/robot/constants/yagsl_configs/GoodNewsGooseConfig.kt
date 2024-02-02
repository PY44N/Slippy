package frc.robot.constants.yagsl_configs

import swervelib.encoders.CANCoderSwerve
import swervelib.imu.Pigeon2Swerve
import swervelib.motors.SparkMaxSwerve
import swervelib.motors.TalonFXSwerve
import swervelib.parser.PIDFConfig
import swervelib.parser.json.MotorConfigDouble

object GoodNewsGooseConfig {
    val imu = Pigeon2Swerve(
        30,
        "",
    )
    val HEADING_PID = PIDFConfig(0.0, 0.0)

    val CONVERSION_FACTORS = MotorConfigDouble(16.8, 0.045777493)

    val DRIVE_PID = PIDFConfig(0.00023, 0.0000002, 1.0)
    val TWIST_PID = PIDFConfig(0.004, 1.5)

    val FRONT_LEFT_DRIVE_MOTOR = TalonFXSwerve(5, true)
    val FRONT_LEFT_TWIST_MOTOR = SparkMaxSwerve(9, false)
    val FRONT_LEFT_ENCODER = CANCoderSwerve(13, "")

    val FRONT_RIGHT_DRIVE_MOTOR = TalonFXSwerve(2, true)
    val FRONT_RIGHT_TWIST_MOTOR = SparkMaxSwerve(6, false)
    val FRONT_RIGHT_ENCODER = CANCoderSwerve(10, "")

    val BACK_RIGHT_DRIVE_MOTOR = SparkMaxSwerve(3, true)
    val BACK_RIGHT_TWIST_MOTOR = SparkMaxSwerve(7, false)
    val BACK_RIGHT_ENCODER = CANCoderSwerve(11, "")

    val BACK_LEFT_DRIVE_MOTOR = SparkMaxSwerve(4, true)
    val BACK_LEFT_TWIST_MOTOR = SparkMaxSwerve(8, false)
    val BACK_LEFT_ENCODER = CANCoderSwerve(12, "")

    val goodNewsGoose = YAGSLConfig(
        imu,
        false,
        .5,
        4.20,
        12.375,
        12.375,
        CONVERSION_FACTORS,
        1.19,
        12.0,
        40,
        20,
        .25,
        .25,
        FRONT_LEFT_DRIVE_MOTOR,
        true,
        FRONT_LEFT_TWIST_MOTOR,
        true,
        FRONT_LEFT_ENCODER,
        296.630859375,
        FRONT_RIGHT_DRIVE_MOTOR,
        false,
        FRONT_RIGHT_TWIST_MOTOR,
        true,
        FRONT_RIGHT_ENCODER,
        146.25,
        BACK_RIGHT_DRIVE_MOTOR,
        true,
        BACK_RIGHT_TWIST_MOTOR,
        true,
        BACK_RIGHT_ENCODER,
        199.86328125,
        BACK_LEFT_DRIVE_MOTOR,
        true,
        BACK_LEFT_TWIST_MOTOR,
        true,
        BACK_LEFT_ENCODER,
        142.91015625,
        DRIVE_PID,
        TWIST_PID,
        HEADING_PID,
    )
}   