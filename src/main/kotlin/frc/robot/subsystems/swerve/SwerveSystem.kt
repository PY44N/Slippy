package frc.robot.subsystems.swerve

import MiscCalculations
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathConstraints
import com.pathplanner.lib.util.HolonomicPathFollowerConfig
import com.pathplanner.lib.util.PIDConstants
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.GlobalZones
import frc.robot.RobotContainer
import frc.robot.constants.DriveConstants
import frc.robot.constants.FieldConstants
import frc.robot.constants.PathPlannerLibConstants
import frc.robot.util.AutoTwistController
import org.littletonrobotics.junction.Logger
import swervelib.SwerveDrive
import swervelib.parser.SwerveParser
import swervelib.telemetry.SwerveDriveTelemetry
import java.io.File
import kotlin.math.PI
import kotlin.math.atan2

class SwerveSystem(private val io: SwerveSystemIO,  config: File) : SubsystemBase() {
    private val inputs: SwerveSystemIO.SwerveSystemIOInputs = SwerveSystemIO.SwerveSystemIOInputs

    var inputRotation: Double = 0.0
    private val autoConstraints: PathConstraints

    private val xPID: PIDController = PIDController(.1, 0.0, 0.01)
    private val yPID: PIDController = PIDController(.1, 0.0, 0.01)

    private val PIDDeadzone = .005;

    val autoTwistController: AutoTwistController = AutoTwistController()

    val swerveDrive: SwerveDrive = try {
            SwerveParser(config).createSwerveDrive(DriveConstants.MAX_SPEED)
        } catch (e: Exception) {
            throw RuntimeException(e)
        }


    init {
        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.NONE

        swerveDrive.setHeadingCorrection(false)
        swerveDrive.setMotorIdleMode(false)
        swerveDrive.pushOffsetsToControllers()
        setupPathPlanner()
        autoConstraints = PathConstraints(
            swerveDrive.maximumVelocity, 4.0,
            swerveDrive.maximumAngularVelocity, Units.degreesToRadians(720.0)
        )

        swerveDrive.setHeadingCorrection(true)
    }

    //Takes in joystick inputs
    fun calculateJoyTranslation(rightX: Double, rightY: Double, throttle: Double, deadzoneX: Double, deadzoneY: Double): Translation2d {
        return Translation2d(
                -MiscCalculations.calculateDeadzone(rightY,deadzoneX) * DriveConstants.MAX_SPEED * throttle,
                -MiscCalculations.calculateDeadzone(rightX,deadzoneY) * DriveConstants.MAX_SPEED * throttle
        )
    }

    fun calculateJoyThrottle(joyThrottle: Double): Double {
        //Milan: trust me bro this'll work totally definitely please don't question it
        return ((joyThrottle * -1) + 1) / 2
    }

    private fun setupPathPlanner() {
        AutoBuilder.configureHolonomic(
            swerveDrive::getPose,
            swerveDrive::resetOdometry,
            swerveDrive::getRobotVelocity,
            this::autoDrive,
            HolonomicPathFollowerConfig(
                PathPlannerLibConstants.translationPID,
                PIDConstants(
                    swerveDrive.swerveController.thetaController.p,
                    swerveDrive.swerveController.thetaController.i,
                    swerveDrive.swerveController.thetaController.d,
                ),
                swerveDrive.maximumVelocity,
                swerveDrive.swerveDriveConfiguration.driveBaseRadiusMeters,
                PathPlannerLibConstants.replanningConfig,
            ),
            this::isRed,
            this,
        )
    }

    fun drive(translation: Translation2d, rotation: Double, fieldRelative: Boolean) {
        inputRotation = rotation
        swerveDrive.drive(translation, rotation, true, false)
    }

//    fun driveSpeakerOriented(translation: Translation2d, angleOffset: Double) {
//        val p = swerveDrive.pose
//        val speakerAngle =
//            atan2(
//                FieldConstants.SPEAKER_CENTER_Y - p.y,
//                FieldConstants.SPEAKER_CENTER_X - p.x
//            ) + angleOffset * PI
//        // figure out sign and stuff
//        swerveDrive.drive(translation, speakerAngle, true, false)
//    }

    fun autoDrive(velocity: ChassisSpeeds) {
        swerveDrive.drive(velocity)
    }

    fun isRed(): Boolean =
        DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red


    //Updates the swerve drive position zone in the state machine
    fun updateGlobalZone() {
        val pos = swerveDrive.pose.translation

        if (MiscCalculations.translation2dWithinRange(pos, RobotContainer.stateMachine.currentRobotZone.range)) {
            return
        }

        val currentRange = MiscCalculations.findMatchingTranslation2dRange(pos, arrayOf(GlobalZones.Wing.range, GlobalZones.NO.range, GlobalZones.Stage.range))

        //Not in any zone
        if (currentRange.first.x == -1.0 && currentRange.second.y == -1.0) {
            println("Not in a valid zone; ERROR")
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
        }
        else {
            println("Not in a valid zone; current zone returned was not default; ERROR")
        }
    }

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("SwerveSystem", inputs)
        Logger.recordOutput("RobotAccel", swerveDrive.accel.orElse(Translation3d(0.0, 0.0, 0.0)))
        Logger.recordOutput("RobotVelocity", swerveDrive.fieldVelocity)
        Logger.recordOutput("RobotRotation", swerveDrive.gyroRotation3d.angle)
        Logger.recordOutput("RobotPose", swerveDrive.pose)


        updateGlobalZone()
        //TODO: update global (specific) positions in the state machine
    }

    fun driveToPose(pose: Pose2d): Command {
        return AutoBuilder.pathfindToPose(
            pose,
            autoConstraints,
            0.0,  // Goal end velocity in meters/sec
            0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
        )
    }
}
