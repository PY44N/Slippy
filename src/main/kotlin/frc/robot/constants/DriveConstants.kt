package frc.robot.constants

object DriveConstants {
    const val FRONT_LEFT_DRIVE: Int = 6
    const val FRONT_RIGHT_DRIVE: Int = 8
    const val BACK_LEFT_DRIVE: Int = 5
    const val BACK_RIGHT_DRIVE: Int = 7

    const val FRONT_LEFT_TWIST: Int = 10
    const val FRONT_RIGHT_TWIST: Int = 12
    const val BACK_LEFT_TWIST: Int = 9
    const val BACK_RIGHT_TWIST: Int = 11

    const val ROBOT_RADIUS: Double =
        0.5 // this is a random number (should be center to (furthest) point of wheel contact)
    const val MAX_SPEED: Double = 3.0 // please find the real value
}