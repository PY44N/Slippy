package frc.robot.util;

import frc.robot.RobotContainer
import frc.robot.constants.FieldConstants
import java.lang.Math.pow
import kotlin.math.acos
import kotlin.math.atan
import kotlin.math.pow
import kotlin.math.sqrt

data class Shot(val shooterAngle: Double, val robotAngle: Double)

public class TargetingSystem {
    val g = 9.81
    val h = FieldConstants.SPEAKER_CENTER_Z
    fun calculateShot(): Shot {
        val robotPose = RobotContainer.swerveSystem.swerveDrive.pose
        val robotVelocity = RobotContainer.swerveSystem.swerveDrive.robotVelocity
        val x = robotPose.x - FieldConstants.SPEAKER_CENTER_X
        val y = robotPose.y - FieldConstants.SPEAKER_CENTER_Y
        val robotAngle = robotPose.rotation.degrees
        val vx = robotVelocity.vxMetersPerSecond
        val vy = robotVelocity.vyMetersPerSecond
        val rSquared = x*x+y*y
        val r = sqrt(rSquared)
        val inverseR = 1.0/r
        val v = sqrt(vx*vx+vy*vy)
        val rDot = inverseR*(x*vx+y*vy)
        val targetRobotAngle = acos(x*((x*x-y*y)*vx+2*x*y*vy)/(r.pow(1.5)*v))
        val targetShooterAngle = atan(h/r + g*r/((1.3+.15*rDot)*v*r/(sqrt(rSquared+h*h)) + rDot).pow(2))
        return Shot(targetShooterAngle, targetRobotAngle)
    }
}
