package frc.robot

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.*
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandJoystick
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.commands.TeleopSwerveDriveCommand
import frc.robot.commands.cannon.AutoAmp
import frc.robot.commands.cannon.AutoIntake
import frc.robot.commands.cannon.AutoShootCommand
import frc.robot.constants.TunerConstants
import frc.robot.subsystems.cannon.CannonIOReal
import frc.robot.subsystems.cannon.CannonSystem
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain
import frc.robot.subsystems.swerve.SwerveSystem
import frc.robot.subsystems.swerve.Telemetry
import frc.robot.subsystems.trunk.TrunkIOReal
import frc.robot.subsystems.trunk.TrunkSystem

object RobotContainer {
    private val MaxSpeed: Double = TunerConstants.kSpeedAt12VoltsMps // kSpeedAt12VoltsMps desired top speed
    private val MaxAngularRate = 1.5 * Math.PI // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */ //  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
    var drivetrain: CommandSwerveDrivetrain = TunerConstants.DriveTrain // My drivetrain

    // driving in open loop
    private val brake = SwerveDriveBrake()
    private val point = PointWheelsAt()
    private val logger: Telemetry = Telemetry(MaxSpeed)

    val leftJoystick: CommandJoystick = CommandJoystick(0)
    val rightJoystick: CommandJoystick = CommandJoystick(1)
    val xboxController: CommandXboxController = CommandXboxController(2)

    val trunkSystem = TrunkSystem(TrunkIOReal())

    val stateMachine: RobotStateMachine = RobotStateMachine()

    val cannonSystem: CannonSystem = CannonSystem(CannonIOReal())

    val autonomousCommand: Command = Commands.run({})

    var teleopSwerveCommand: Command = TeleopSwerveDriveCommand()

    val intakeLimelight: String = "limelight-intake"

    val autoStateManagementEnableButton: Boolean
        get() = SmartDashboard.getBoolean("Enable Automatic State Management", false)

    val robotActionSendable: SendableChooser<RobotAction> = SendableChooser<RobotAction>()

    val swerveSystem: SwerveSystem = SwerveSystem()

    init {
        configureBindings()
    }

    private fun configureBindings() {
        xboxController.a().toggleOnTrue(AutoIntake())
        xboxController.x().toggleOnTrue(AutoShootCommand())
//        xboxController.x().onTrue(Commands.runOnce({
//            println("x button pressed")
//            cannonSystem.shoot()
//        }))
        xboxController.b().onTrue(Commands.runOnce({
            cannonSystem.killShooter()
        }))
        xboxController.y().toggleOnTrue(AutoAmp())
        //MURDER...KILL IT ALL
        xboxController.start().onTrue(Commands.runOnce({
            cannonSystem.killShooter()
            cannonSystem.killIntake()
        }))
    }


//    val autoChooser: SendableChooser<Command> = AutoBuilder.buildAutoChooser()
//        SmartDashboard.putData("Auto Chooser", autoChooser)


}

