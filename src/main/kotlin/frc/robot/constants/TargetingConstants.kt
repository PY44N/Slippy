package frc.robot.constants

import edu.wpi.first.math.util.Units

object TargetingConstants {
    // shooter velocity transfer proportion
    var velocityMultiplier = 1.0

    // coords of point we're aiming at relative to center of base of the speaker board (board with the fiducials)
    var endpointX = .2
    var endpointY = 0.0
    var endpointZ = 2.15

    // coords of center of speaker backboard
    var speakerX = 0.0
    var speakerY = Units.inchesToMeters(104.861)

    // height that we shoot from; technically varies a bit but lets just say it doesnt
    var shooterZ =Units.inchesToMeters(25.0)

}