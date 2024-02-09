package frc.robot.constants

import com.pathplanner.lib.util.PIDConstants
import com.pathplanner.lib.util.ReplanningConfig

object PathPlannerLibConstants {
    val rotationPID = PIDConstants(0.0, 0.0)

    // TODO: figure out if we need to change anything here
    val replanningConfig = ReplanningConfig()

    // TODO: tune
    val translationPID = PIDConstants(5.0, 0.0, 0.0)
}