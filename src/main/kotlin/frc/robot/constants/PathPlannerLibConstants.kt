package frc.robot.constants

import com.pathplanner.lib.util.PIDConstants
import com.pathplanner.lib.util.ReplanningConfig

object PathPlannerLibConstants {
    val replanningConfig = ReplanningConfig()
    val translationPID = PIDConstants(0.0, 0.0)
    val rotationPID = PIDConstants(0.0, 0.0)
}