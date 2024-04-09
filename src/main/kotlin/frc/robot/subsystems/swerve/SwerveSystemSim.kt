package frc.robot.subsystems.swerve

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Nat
import edu.wpi.first.math.Vector
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N2
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.util.Timer
import kotlin.math.cos
import kotlin.math.sin

class SwerveSystemSim(private var robotPose: Pose2d = Pose2d()) : SubsystemBase(), GenericSwerveSystem {
    private var currentSpeed = ChassisSpeeds()
    var lastLoopTime = Timer.getFPGATimestamp()

    override val currentRobotChassisSpeeds: ChassisSpeeds
        get() = currentSpeed

    override fun getSwervePose(): Pose2d = robotPose

    override fun zeroGyro() {
        robotPose = robotPose.rotateBy(robotPose.rotation.unaryMinus())
    }

    override fun getGyroRotation(): Double {
        TODO("Not yet implemented")
    }

    override fun setGyroRotation(rotation: Double) {
        TODO("Not yet implemented")
    }

    override fun swerveState(): SwerveDrivetrain.SwerveDriveState {
        TODO("Not yet implemented")
    }

    override fun applyRobotRelativeDriveRequest(x: Double, y: Double, rotation: Double): Command {
        return run {
            currentSpeed = ChassisSpeeds(x, y, rotation)
        }
    }

    override fun applyDriveRequest(x: Double, y: Double, rotation: Double): Command {
        return run {
            currentSpeed =
                ChassisSpeeds(x, y, rotation)
        }
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

        robotPose = robotPose.plus(
            Transform2d(
                currentSpeed.vxMetersPerSecond * dtSeconds,
                currentSpeed.vyMetersPerSecond * dtSeconds,
                Rotation2d(currentSpeed.omegaRadiansPerSecond * dtSeconds)
            )
        )

        SmartDashboard.putNumber("Robot ChassisSpeed X", currentSpeed.vxMetersPerSecond)
        SmartDashboard.putNumber("Robot ChassisSpeed Y", currentSpeed.vyMetersPerSecond)

        lastLoopTime = currentTime
    }
}