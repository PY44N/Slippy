package frc.robot.constants

import com.pathplanner.lib.util.HolonomicPathFollowerConfig
import com.pathplanner.lib.util.PIDConstants
import com.pathplanner.lib.util.ReplanningConfig

object PathPlannerLibConstants {
    // TODO: figure out if we need to change anything here
    var replanningConfig = ReplanningConfig()
    // TODO: tune
    var translationPID = PIDConstants(5.0, 0.0, 0.0)
    var rotationPID = PIDConstants(
        DriveConstants.HEADING_PID.p,
        DriveConstants.HEADING_PID.i,
        DriveConstants.HEADING_PID.d,
        )
    var pathPlannerConfig = HolonomicPathFollowerConfig(
        translationPID,
        rotationPID,
        DriveConstants.MAX_SPEED,
        DriveConstants.DRIVE_CONFIG.driveBaseRadiusMeters,
        replanningConfig,
    )
}