package frc.robot.constants

import com.pathplanner.lib.util.HolonomicPathFollowerConfig
import com.pathplanner.lib.util.PIDConstants
import com.pathplanner.lib.util.ReplanningConfig
import frc.robot.constants.DriveConstants

object PathPlannerLibConstants {
    // TODO: figure out if we need to change anything here
    var replanningConfig = ReplanningConfig()
    // TODO: tune
    var translationPID = PIDConstants(5.0, 0.0, 0.0)
    var rotationPID = PIDConstants(
        DriveConstants.headingPID.p,
        DriveConstants.headingPID.i,
        DriveConstants.headingPID.d,
        )
    var pathPlannerConfig = HolonomicPathFollowerConfig(
        translationPID,
        rotationPID,
        DriveConstants.MAX_SPEED,
        DriveConstants.driveConfig.driveBaseRadiusMeters,
        replanningConfig,
    )
}