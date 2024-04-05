package frc.robot.subsystems.swerve

import edu.wpi.first.math.Matrix
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.wpilibj2.command.Command

interface GenericSwerveSystem {
    val currentRobotChassisSpeeds: ChassisSpeeds

    fun getSwervePose(): Pose2d

    fun calculateJoyTranslation(
        rightX: Double,
        rightY: Double,
        throttle: Double,
        deadzoneX: Double,
        deadzoneY: Double
    ): Translation2d

    fun calculateJoyThrottle(joyThrottle: Double): Double

    fun zeroGyro()

    fun applyRobotRelativeDriveRequest(x: Double, y: Double, rotation: Double): Command
    fun applyDriveRequest(x: Double, y: Double, rotation: Double): Command

    fun addVisionMeasurement(visionRobotPoseMeters: Pose2d, timestampSeconds: Double)
    fun setVisionMeasurementStdDevs(visionMeasurementStdDevs: Matrix<N3, N1>)

    fun getAutoPath(name: String): Command
}