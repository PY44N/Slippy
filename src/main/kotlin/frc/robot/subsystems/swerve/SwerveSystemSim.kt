package frc.robot.subsystems.swerve

import edu.wpi.first.math.Matrix
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.util.Timer

class SwerveSystemSim(private var robotPose: Pose2d = Pose2d()) : SubsystemBase(), GenericSwerveSystem {
    private var currentSpeed = ChassisSpeeds()
    var lastLoopTime = Timer.getFPGATimestamp()

    override val currentRobotChassisSpeeds: ChassisSpeeds
        get() = currentSpeed

    override fun getSwervePose(): Pose2d = robotPose

    override fun zeroGyro() {
        robotPose = robotPose.rotateBy(robotPose.rotation.unaryMinus())
    }

    override fun applyRobotRelativeDriveRequest(x: Double, y: Double, rotation: Double): Command {
        return Commands.runOnce({ println("Trying to drive robot oriented (not implemented yet)") })
    }

    override fun applyDriveRequest(x: Double, y: Double, rotation: Double): Command {
        return Commands.runOnce({ currentSpeed = ChassisSpeeds(x, y, rotation) })
    }

    // For out simulation use case we probably don't need to implement these
    override fun addVisionMeasurement(visionRobotPoseMeters: Pose2d, timestampSeconds: Double) {}
    override fun setVisionMeasurementStdDevs(visionMeasurementStdDevs: Matrix<N3, N1>) {}

    override fun getAutoPath(name: String): Command {
        TODO("Not yet implemented")
    }

    override fun periodic() {
        val currentTime = Timer.getFPGATimestamp()
        val dtSeconds = currentTime - lastLoopTime

        robotPose.plus(
            Transform2d(
                currentSpeed.vxMetersPerSecond * dtSeconds,
                currentSpeed.vyMetersPerSecond * dtSeconds,
                Rotation2d(currentSpeed.omegaRadiansPerSecond * dtSeconds)
            )
        )

        lastLoopTime = currentTime
    }
}