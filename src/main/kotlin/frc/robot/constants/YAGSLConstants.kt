package frc.robot.constants

import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.util.Units

object YAGSLConstants {
    // BAD VALUES
    // module positions (center of robot to center of wheel)
    val kinematics = SwerveDriveKinematics(
        Translation2d( // Front Left
            Units.inchesToMeters(12.5),
            Units.inchesToMeters(12.5),
        ),
        Translation2d( // Front Right
            Units.inchesToMeters(12.5),
            Units.inchesToMeters(-12.5),
        ),
        Translation2d( // Back Left
            Units.inchesToMeters(-12.5),
            Units.inchesToMeters(12.5),
        ),
        Translation2d( // Back Right
            Units.inchesToMeters(-12.5),
            Units.inchesToMeters(-12.5),
        ),
    )

}