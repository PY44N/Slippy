package frc.robot.constants

object CannonConstants {

    //    val SHOOTER_MAX_RPM = 7000.0
    val SHOOTER_MAX_RPM = 7000.0
    val INTAKE_STOW_DELAY = .3

    val OUTER_INTAKE_PERCENT: Double = -.8
    val INNER_INTAKE_PERCENT: Double = .8

    val OUTER_FEED_PERCENT: Double = -.8
    val INNER_FEED_PERCENT: Double = .8

    val OUTER_SPIT_PERCENT: Double = .8
    val INNER_SPIT_PERCENT: Double = -.8

    var OUTER_AMP_PERCENT: Double = -.7
    var INNER_AMP_PERCENT: Double = -.7

    // rpm
    const val SHOOTER_SHOOT_VELOCITY: Double = 4010.2032905044503

    const val SHOOTER_PREP_VELOCITY: Double = .8 * SHOOTER_SHOOT_VELOCITY

    const val SHOOTER_VELOCITY_DEADZONE: Double = 150.0
    val SHOOTER_YAW_DEADZONE: Double = 1.0
    val SHOOTER_PITCH_DEADZONE: Double = 1.0

    //measure in ms
    val NOTE_EXIT_BEAMBREAK_DELAY: Double = 1.0

    val LEFT_SHOOTER_MOTOR_ID = 40
    val RIGHT_SHOOTER_MOTOR_ID = 41
    val OUTER_INTAKE_MOTOR_ID = 50
    val INNER_INTAKE_MOTOR_ID = 51

    val outerIntakePercent = .5
    val innerIntakePercent = .5

    var shooterKP = 1.2
    var shooterKI = 0.0
    var shooterKD = 0.0
    var ShooterIz = 0.0
    var ShooterFF = 0.0
    var ShooterMax = -1.0
    var ShooterMin = 1.0
}
