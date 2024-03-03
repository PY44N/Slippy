//package frc.robot.util;
//
//import frc.robot.RobotContainer
//import frc.robot.constants.FieldConstants
//import frc.robot.constants.TrunkConstants
//import kotlin.math.*
//
//data class ShotSetup(val robotAngle: Double, val shooterAngle: Double)
//private data class TargetingVariables(val underStage: Boolean) {
//    val x: Double
//    val y: Double
//    val vx: Double
//    val vy: Double
//    val r: Double
//    val rSquared: Double
//
//    init {
//        val robotPose = RobotContainer.swerveSystem.getSwervePose()
//        val robotVelocity = RobotContainer.swerveSystem.driveTrain.currentRobotChassisSpeeds
//        val robotAngle = robotPose.rotation.radians
//
//        val shootingOffset = if (underStage) TrunkConstants.UNDER_STAGE_SHOOTING_OFFSET else TrunkConstants.SHOOTING_OFFSET
//        val xOffset = FieldConstants.SPEAKER_CENTER_X + cos(robotAngle) * shootingOffset
//        val yOffset = FieldConstants.SPEAKER_CENTER_Y + sin(robotAngle) * shootingOffset
//
//        x = robotPose.x - xOffset
//        y = robotPose.y - yOffset
//        vx = robotVelocity.vxMetersPerSecond
//        vy = robotVelocity.vyMetersPerSecond
//        rSquared = x * x + y * y
//        r = sqrt(rSquared)
//    }
//}
//
//class TargetingSystem {
//    private val g = 9.81
//    private val rad2deg = 180.0 / PI
//    val shootingVelocity = 15.0
//    private val shootingVelocityScaling = 1.3
//    fun calculateShot(underStage: Boolean): ShotSetup {
//        val shooterVars = TargetingVariables(underStage)
//
//        val targetRobotAngle = robotAngleFunction(shooterVars)
//        val targetShooterAngle = shooterAngleFunction(shooterVars)
//
//        return ShotSetup(targetRobotAngle, targetShooterAngle)
//    }
//
//    fun calculateShootingAngle(underStage: Boolean) = shooterAngleFunction(TargetingVariables(underStage))
//
//    fun calculateRobotAngle(underStage: Boolean) = robotAngleFunction(TargetingVariables(underStage))
//
//    private fun robotAngleFunction(vars: TargetingVariables): Double {
//        return acos(vars.x * ((vars.x * vars.x - vars.y * vars.y) * vars.vx + 2 * vars.x * vars.y * vars.vy) /
//                (vars.r.pow(1.5) * sqrt(vars.vx * vars.vx + vars.vy * vars.vy))) * rad2deg
//    }
//
//    private fun shooterAngleFunction(vars: TargetingVariables): Double {
//        val inverseR = 1.0 / vars.r
//        val rDot = inverseR * (vars.x * vars.vx + vars.y * vars.vy)
//        val h = FieldConstants.SPEAKER_CENTER_Z - if (vars.underStage)
//            TrunkConstants.UNDER_STAGE_SHOOTING_HEIGHT else TrunkConstants.SHOOTING_HEIGHT
//
//        return atan(h * inverseR + g * vars.r /
//                ((shootingVelocityScaling + .15 * rDot) * shootingVelocity * vars.r / (sqrt(vars.rSquared + h * h)) + rDot).pow(2)) * rad2deg +
//                (40.0 * rDot) / (shootingVelocityScaling * shootingVelocity)
//    }
//
//    fun getShotNoVelocity(underStage: Boolean): ShotSetup {
//        val vars = TargetingVariables(underStage)
//        val h = FieldConstants.SPEAKER_CENTER_Z - if (vars.underStage)
//            TrunkConstants.UNDER_STAGE_SHOOTING_HEIGHT else TrunkConstants.SHOOTING_HEIGHT
//
//        val targetRobotAngle = acos(vars.x / vars.r) * rad2deg
//        val targetShooterAngle = atan2(g * (vars.rSquared + h * h) + 2 * h * shootingVelocity, vars.r) * rad2deg
//
//        return ShotSetup(targetRobotAngle, targetShooterAngle)
//    }
//}
