package frc.robot

import com.ctre.phoenix6.Utils
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.*
import com.pathplanner.lib.auto.AutoBuilder
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandJoystick
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.constants.TunerConstants
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain
import frc.robot.subsystems.swerve.Telemetry


/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
object RobotContainer {
    // The robot's subsystems and commands are defined here...
    val rightJoystick: CommandJoystick = CommandJoystick(1)
    private val xboxController: CommandXboxController = CommandXboxController(2)


    lateinit var teleopSwerveCommand: Command
    val autonomousCommand: Command = Commands.run({})

    val autoChooser: SendableChooser<Command>

    private const val MaxSpeed = TunerConstants.kSpeedAt12VoltsMps // kSpeedAt12VoltsMps desired top speed
    private const val MaxAngularRate = 1.5 * Math.PI // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private val joystick = CommandXboxController(0) // My joystick
    private val drivetrain: CommandSwerveDrivetrain = TunerConstants.DriveTrain // My drivetrain

    private val drive: FieldCentric = FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // I want field-centric

    // driving in open loop
    private val brake = SwerveDriveBrake()
    private val point = PointWheelsAt()
    private val logger = Telemetry(MaxSpeed)

    /**
     * The container for the robot.  Contains subsystems, IO devices, and commands.
     */
    init {
        autoChooser = AutoBuilder.buildAutoChooser()
        SmartDashboard.putData("Auto Chooser", autoChooser)

        // Configure the button bindings
        configureButtonBindings()
    }

    /**
     * Use this method to define your button->command mappings.  Buttons can be created by
     * instantiating a [GenericHID] or one of its subclasses ([ ] or [XboxController]), and then passing it to a
     * [edu.wpi.first.wpilibj2.command.button.JoystickButton].
     */
    private fun configureButtonBindings() {
//        teleopSwerveCommand = Commands.run(
//            {
//                swerveSystem.drive(
//                    Translation2d(
//                        (if (abs(rightJoystick.y) > 0.15) -rightJoystick.y * DriveConstants.MAX_SPEED else 0.0),
//                        (if (abs(rightJoystick.x) > 0.15) -rightJoystick.x * DriveConstants.MAX_SPEED else 0.0)
//                    ),
//                    (if (abs(rightJoystick.twist) > 0.15) -rightJoystick.twist * DriveConstants.MAX_ANGLE_SPEED else 0.0),
//                    true
//                )
//            },
//            swerveSystem
//        )

//        rightJoystick.button(2).onTrue(Commands.run({ swerveSystem.swerveDrive.zeroGyro() }))
        drivetrain.defaultCommand = drivetrain.applyRequest {
            drive.withVelocityX(-joystick.leftY * MaxSpeed) // Drive forward with
                    // negative Y (forward)
                    .withVelocityY(-joystick.leftX * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.rightX * MaxAngularRate)
        } // Drive counterclockwise with negative X (left)


        joystick.a().whileTrue(drivetrain.applyRequest { brake })
        joystick.b().whileTrue(drivetrain
                .applyRequest {
                    point.withModuleDirection(
                            Rotation2d(
                                    -joystick.leftY,
                                    -joystick.leftX
                            )
                    )
                })


        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce { drivetrain.seedFieldRelative() })

        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(Pose2d(Translation2d(), Rotation2d.fromDegrees(90.0)))
        }
        drivetrain.registerTelemetry { state: SwerveDriveState ->
            logger.telemeterize(
                    state
            )
        }
    }
}