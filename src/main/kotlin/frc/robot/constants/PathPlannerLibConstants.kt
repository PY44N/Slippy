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
        YAGSLConfig.controllerConfig.headingPIDF.p,
        YAGSLConfig.controllerConfig.headingPIDF.i,
        YAGSLConfig.controllerConfig.headingPIDF.d,
    )
    var pathPlannerConfig = HolonomicPathFollowerConfig(
        translationPID,
        rotationPID,
        YAGSLConfig.maxSpeedMPS,
        YAGSLConfig.driveConfig.driveBaseRadiusMeters,
        replanningConfig,
    )
}