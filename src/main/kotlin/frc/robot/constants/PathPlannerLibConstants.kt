package frc.robot.constants

import com.pathplanner.lib.util.HolonomicPathFollowerConfig
import com.pathplanner.lib.util.PIDConstants
import com.pathplanner.lib.util.ReplanningConfig

object PathPlannerLibConstants {
    var replanningConfig = ReplanningConfig()
    var translationPID = PIDConstants(0.0, 0.0)
    var rotationPID = PIDConstants(0.0, 0.0)
    var pathPlannerConfig = HolonomicPathFollowerConfig(
        translationPID,
        rotationPID,
        DriveConstants.MAX_SPEED,
        DriveConstants.DRIVE_CONFIG.driveBaseRadiusMeters,
        replanningConfig,
    )
}