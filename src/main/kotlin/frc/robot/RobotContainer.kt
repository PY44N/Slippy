package frc.robot

import com.pathplanner.lib.auto.AutoBuilder
import edu.wpi.first.networktables.GenericEntry
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandJoystick
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.commands.TeleopSwerveDriveCommand
import frc.robot.subsystems.cannon.CannonIOReal
import frc.robot.subsystems.cannon.CannonSystem
import frc.robot.subsystems.swerve.SwerveSystem
import frc.robot.subsystems.swerve.SwerveSystemIOReal
import frc.robot.subsystems.trunk.TrunkIOReal
import frc.robot.subsystems.trunk.TrunkIOSim
import frc.robot.subsystems.trunk.TrunkSystem
import java.io.File

object RobotContainer {
    val leftJoystick: CommandJoystick = CommandJoystick(0)
    val rightJoystick: CommandJoystick = CommandJoystick(1)
    val xboxController: CommandXboxController = CommandXboxController(2)

    val swerveSystem: SwerveSystem
    val trunkSystem = TrunkSystem(TrunkIOReal())

    val stateMachine: RobotStateMachine = RobotStateMachine()

    val cannonSystem: CannonSystem = CannonSystem(CannonIOReal())

    val autonomousCommand: Command = Commands.run({})

    lateinit var teleopSwerveCommand: Command


//    val autoChooser: SendableChooser<Command> = AutoBuilder.buildAutoChooser()

    val intakeLimelight: String = "limelight-intake"

    val autoStateManagementEnableButton: Boolean
            get() = SmartDashboard.getBoolean("Enable Automatic State Management", true)

    val robotActionSendable: SendableChooser<RobotAction> = SendableChooser<RobotAction>()


    /**
     * The container for the robot.  Contains subsystems, IO devices, and commands.
     */
    init {
        when (Constants.currentMode) {
            Constants.Mode.REAL -> {
                swerveSystem = SwerveSystem(
                        SwerveSystemIOReal(),
                        File(Filesystem.getDeployDirectory(), "yagsl_configs/slippy_kraken")
                )
            }

            Constants.Mode.SIM -> {
                // change these later
                swerveSystem = SwerveSystem(
                        SwerveSystemIOReal(),
                        File(Filesystem.getDeployDirectory(), "yagsl_configs/slippy_kraken")
                )
            }

            Constants.Mode.REPLAY -> {
                // change these later
                swerveSystem = SwerveSystem(
                        SwerveSystemIOReal(),
                        File(Filesystem.getDeployDirectory(), "yagsl_configs/slippy_kraken")
                )
            }
        }
//        SmartDashboard.putData("Auto Chooser", autoChooser)

        teleopSwerveCommand = TeleopSwerveDriveCommand()
        teleopSwerveCommand.schedule()

        configureButtonBindings()
    }

    private fun configureButtonBindings() {
    }
}
