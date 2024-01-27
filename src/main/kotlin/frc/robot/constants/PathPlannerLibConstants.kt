package frc.robot.constants

import com.pathplanner.lib.util.HolonomicPathFollowerConfig
import com.pathplanner.lib.util.PIDConstants
import com.pathplanner.lib.util.ReplanningConfig
import frc.robot.RobotContainer

object PathPlannerLibConstants {
    val replanningConfig = ReplanningConfig()
    val translationPID = PIDConstants(0.0, 0.0)
    val rotationPID = PIDConstants(0.0, 0.0)
    val pathPlannerConfig = HolonomicPathFollowerConfig(
        translationPID,
        rotationPID,
        DriveConstants.MAX_SPEED,
        RobotContainer.swerveSystem.swerveDrive.swerveDriveConfiguration.driveBaseRadiusMeters,
        replanningConfig,
    )
}