package frc.robot.subsystems.swerve

import MiscCalculations
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.GlobalZones
import frc.robot.RobotContainer
import frc.robot.constants.DriveConstants
import frc.robot.constants.TunerConstants
import frc.robot.util.AutoTwistController

class SwerveSystemReal() : SubsystemBase(), GenericSwerveSystem {
    private val driveTrain: CommandSwerveDrivetrain = TunerConstants.DriveTrain

    var inputRotation: Double = 0.0

    var gyroOffset = 0.0

    private val xPID: PIDController = PIDController(.1, 0.0, 0.01)
    private val yPID: PIDController = PIDController(.1, 0.0, 0.01)

    private val PIDDeadzone = .005;

    override val currentRobotChassisSpeeds
        get() = driveTrain.currentRobotChassisSpeeds

    private val driveRobotRelative = SwerveRequest.RobotCentric()
        .withDeadband(DriveConstants.MAX_SPEED * 0.1)
        .withRotationalDeadband(DriveConstants.MAX_ANGLE_SPEED * 0.1) // Add a 10% deadband
        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)

    private val drive: SwerveRequest.FieldCentric = SwerveRequest.FieldCentric()
        .withDeadband(DriveConstants.MAX_SPEED * 0.1)
        .withRotationalDeadband(DriveConstants.MAX_ANGLE_SPEED * 0.1) // Add a 10% deadband
        .withDriveRequestType(SwerveModule.DriveRequestType.Velocity); // I want field-centric

    // driving in open loop
    val brake: SwerveRequest.SwerveDriveBrake = SwerveRequest.SwerveDriveBrake();
    val forwardStraight: SwerveRequest.RobotCentric =
        SwerveRequest.RobotCentric().withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);
    val point: SwerveRequest.PointWheelsAt = SwerveRequest.PointWheelsAt();

    /* Path follower */
//    val runAuto: Command = driveTrain.getAutoPath("Tests");


    val autoTwistController: AutoTwistController = AutoTwistController()

    override fun getSwervePose() = driveTrain.state.Pose ?: Pose2d()

    override fun zeroGyro() = driveTrain.seedFieldRelative()

    override fun getGyroRotation(): Double = MiscCalculations.clampAngleTo180(-driveTrain.pigeon2.angle - gyroOffset)

    override fun swerveState(): SwerveDrivetrain.SwerveDriveState {
        return driveTrain.state
    }

    override fun setGyroRotation(rotation: Double) {
        if (gyroOffset == 0.0) {
            gyroOffset = -driveTrain.pigeon2.angle - rotation
        }
    }

    //Updates the swerve drive position zone in the state machine
    fun updateGlobalZone() {
        val pos = getSwervePose().translation

        if (MiscCalculations.translation2dWithinRange(pos, RobotContainer.stateMachine.currentRobotZone.range)) {
            return
        }

        SmartDashboard.putNumber("Robot Pos X", pos.x)
        SmartDashboard.putNumber("Robot Pos Y", pos.y)

        val currentRange = MiscCalculations.findMatchingTranslation2dRange(
            pos,
            arrayOf(GlobalZones.Wing.range, GlobalZones.NO.range, GlobalZones.Stage.range)
        )

        //Not in any zone
        if (currentRange.first.x == -1.0 && currentRange.second.y == -1.0) {
//            println("Not in a valid zone; ERROR")
            return
        }

        val currentZone = when (currentRange) {
            GlobalZones.Stage.range -> GlobalZones.Stage
            GlobalZones.Wing.range -> GlobalZones.Wing
            GlobalZones.NO.range -> GlobalZones.NO
            else -> null
        }

        if (currentZone != null) {
            RobotContainer.stateMachine.prevRobotZone = RobotContainer.stateMachine.currentRobotZone
            RobotContainer.stateMachine.currentRobotZone = currentZone
        } else {
//            println("Not in a valid zone; current zone returned was not default; ERROR")
        }
    }

    /*
        public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }
     */

    override fun applyRobotRelativeDriveRequest(x: Double, y: Double, rotation: Double): Command {
        return driveTrain.applyRequest {
            driveRobotRelative.withVelocityX(-x).withVelocityY(-y).withRotationalRate(rotation)
        }
    }

    override fun applyDriveRequest(x: Double, y: Double, rotation: Double): Command {
        return if (DriverStation.getAlliance().isPresent && DriverStation.getAlliance()
                .get() == DriverStation.Alliance.Red
        ) {
            driveTrain.applyRequest { drive.withVelocityX(-x).withVelocityY(-y).withRotationalRate(rotation) }
        } else {
            driveTrain.applyRequest { drive.withVelocityX(x).withVelocityY(y).withRotationalRate(rotation) }

        }
    }

    override fun addVisionMeasurement(visionRobotPoseMeters: Pose2d, timestampSeconds: Double) {
        driveTrain.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds)
    }

    override fun setVisionMeasurementStdDevs(visionMeasurementStdDevs: Matrix<N3, N1>) {
        driveTrain.setVisionMeasurementStdDevs(visionMeasurementStdDevs)
    }

    override fun getAutoPath(name: String): Command {
        return driveTrain.getAutoPath(name)
    }

    override fun periodic() {
        updateGlobalZone()
    }

    init {
        driveTrain.daqThread.setThreadPriority(99)
    }
}
