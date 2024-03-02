package frc.robot

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.*
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.constants.TunerConstants
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain
import frc.robot.subsystems.swerve.Telemetry

object RobotContainer {
    private val MaxSpeed: Double = TunerConstants.kSpeedAt12VoltsMps // kSpeedAt12VoltsMps desired top speed
    private val MaxAngularRate = 1.5 * Math.PI // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */ //  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
    var drivetrain: CommandSwerveDrivetrain = TunerConstants.DriveTrain // My drivetrain

    val drive: FieldCentric = FieldCentric()
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // I want field-centric

    // driving in open loop
    private val brake = SwerveDriveBrake()
    private val point = PointWheelsAt()
    private val logger: Telemetry = Telemetry(MaxSpeed)

    private fun configureBindings() {
//    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
//        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
//                                                                                           // negative Y (forward)
//            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
//            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
//        ));
//
//    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
//    joystick.b().whileTrue(drivetrain
//        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));
//
//    // reset the field-centric heading on left bumper press
//    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
//
//    if (Utils.isSimulation()) {
//      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
//    }
//    drivetrain.registerTelemetry(logger::telemeterize);
    }

    val autonomousCommand: Command
        get() = Commands.print("No autonomous command configured")
}
