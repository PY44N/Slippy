package frc.robot.subsystems.swerve

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.commands.PathPlannerAuto
import com.pathplanner.lib.util.HolonomicPathFollowerConfig
import com.pathplanner.lib.util.PIDConstants
import com.pathplanner.lib.util.ReplanningConfig
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
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.constants.TunerConstants
import frc.robot.util.Timer
import kotlin.math.cos
import kotlin.math.sin

class SwerveSystemSim(private var robotPose: Pose2d = Pose2d()) : SubsystemBase(), GenericSwerveSystem {
    private var currentSpeed = ChassisSpeeds()
    var lastLoopTime = Timer.getFPGATimestamp()

    init {
        configurePathPlanner()
    }

    private fun configurePathPlanner() {
//        AutoBuilder.configureHolonomic(
//            () -> this.getState().Pose, // Supplier of current robot pose
//        this::seedFieldRelative,  // Consumer for seeding pose against auto
//        this::getCurrentRobotChassisSpeeds,
//        (speeds) -> this.setControl(AutoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
//        new HolonomicPathFollowerConfig (new PIDConstants (5, 0, 0.0), // <- Translation
//        new PIDConstants (1, 0, 0.1), // <- Rotation
//        TunerConstants.kSpeedAt12VoltsMps,
//        driveBaseRadius,
//        new ReplanningConfig ()),
//        () -> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red, // Change this if the path needs to be flipped on red vs blue
//        this);
        AutoBuilder.configureHolonomic(
            { getSwervePose() },
            { zeroGyro() },
            { currentRobotChassisSpeeds },
            { speeds -> currentSpeed = speeds },
            HolonomicPathFollowerConfig(
                PIDConstants(5.0, 0.0, 0.0),
                PIDConstants(1.0, 0.0, 0.1),
                TunerConstants.kSpeedAt12VoltsMps,
                0.0,
                ReplanningConfig()
            ),
            {
                DriverStation.getAlliance().isPresent && DriverStation.getAlliance()
                    .get() == DriverStation.Alliance.Red
            }, this
        )
    }

    override val currentRobotChassisSpeeds: ChassisSpeeds
        get() = currentSpeed

    override fun getSwervePose(): Pose2d = robotPose

    override fun zeroGyro() {
        robotPose = robotPose.rotateBy(robotPose.rotation.unaryMinus())
    }

    override fun getGyroRotationDegrees(): Double {
        return MiscCalculations.clampAngleTo180(robotPose.rotation.degrees)
    }

    override fun setGyroRotationDegrees(rotation: Double) {
        robotPose = robotPose.rotateBy(Rotation2d(Math.toRadians(rotation)).rotateBy(robotPose.rotation.unaryMinus()))
    }

    override fun swerveState(): SwerveDrivetrain.SwerveDriveState {
        // TODO: Maybe simulate SwerveDriveState (for logging?)
        return SwerveDrivetrain.SwerveDriveState()
    }

    override fun applyRobotRelativeDriveRequest(x: Double, y: Double, rotationRadians: Double): Command {
        return run {
            currentSpeed = ChassisSpeeds(x, y, rotationRadians)
        }
    }

    override fun applyDriveRequest(x: Double, y: Double, rotationRadians: Double): Command {
        return run {
            currentSpeed =
                ChassisSpeeds(x, y, rotationRadians)
        }
    }

    // For out simulation use case we probably don't need to implement these
    override fun addVisionMeasurement(visionRobotPoseMeters: Pose2d, timestampSeconds: Double) {}
    override fun setVisionMeasurementStdDevs(visionMeasurementStdDevs: Matrix<N3, N1>) {}

    override fun getAutoPath(name: String): Command {
        return PathPlannerAuto(name)
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