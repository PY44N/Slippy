package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import frc.robot.RobotContainer
import frc.robot.constants.FieldConstants
import frc.robot.constants.TrunkConstants
import java.lang.Math.pow
import kotlin.math.*

data class Shot(val shooterAngle: Double, val robotAngle: Double)
private data class TargetingVariables(val underStage: Boolean) {
    val x: Double
    val y: Double
    val vx: Double
    val vy: Double
    val v: Double
    val r: Double
    val rSquared: Double
    init {
        val robotPose = RobotContainer.swerveSystem.swerveDrive.pose
        val robotVelocity = RobotContainer.swerveSystem.swerveDrive.robotVelocity
        val robotAngle = robotPose.rotation.radians

        val shootingOffset = if(underStage) TrunkConstants.UNDER_STAGE_SHOOTING_OFFSET else TrunkConstants.SHOOTING_OFFSET
        val xOffset = FieldConstants.SPEAKER_CENTER_X + cos(robotAngle) * shootingOffset
        val yOffset = FieldConstants.SPEAKER_CENTER_Y + sin(robotAngle) * shootingOffset

        x = robotPose.x - xOffset
        y = robotPose.y - yOffset
        vx = robotVelocity.vxMetersPerSecond
        vy = robotVelocity.vyMetersPerSecond
        rSquared = x*x+y*y
        r = sqrt(rSquared)
        v = sqrt(vx*vx+vy*vy)

    }
}

class TargetingSystem {
    val g = 9.81
    val rad2deg = 180.0 / PI

    fun calculateShot(underStage: Boolean): Shot {
        val shooterVars = TargetingVariables(underStage)

        val targetRobotAngle = robotAngleFunction(shooterVars)
        val targetShooterAngle = shooterAngleFunction(shooterVars)

        return Shot(targetShooterAngle, targetRobotAngle)
    }

    fun calculateShootingAngle(underStage: Boolean) = shooterAngleFunction(TargetingVariables(underStage))

    fun calculateRobotAngle(underStage: Boolean) = robotAngleFunction(TargetingVariables(underStage))

    private fun robotAngleFunction(vars: TargetingVariables) : Double {
        return acos(vars.x*((vars.x*vars.x-vars.y*vars.y)*vars.vx+2*vars.x*vars.y*vars.vy)/
                (vars.r.pow(1.5)*vars.v)) * rad2deg
    }

    private fun shooterAngleFunction(vars: TargetingVariables) : Double {
        val inverseR = 1.0/vars.r
        val rDot = inverseR*(vars.x*vars.vx+vars.y*vars.vy)
        val h = FieldConstants.SPEAKER_CENTER_Z - if(vars.underStage)
            TrunkConstants.UNDER_STAGE_SHOOTING_HEIGHT else TrunkConstants.SHOOTING_HEIGHT

        return atan(h*inverseR + g*vars.r/
                ((1.3+.15*rDot)*vars.v*vars.r/(sqrt(vars.rSquared+h*h)) + rDot).pow(2)) * rad2deg
    }
}
