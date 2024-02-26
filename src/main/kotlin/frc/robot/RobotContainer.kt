package frc.robot

import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandJoystick
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.commands.TeleopSwerveDriveCommand
import frc.robot.subsystems.SwerveSystem
import frc.robot.subsystems.SwerveSystemIOReal
import frc.robot.subsystems.trunk.TrunkIOReal
import frc.robot.subsystems.trunk.TrunkIOSim
import frc.robot.subsystems.trunk.TrunkSystem
import java.io.File

object RobotContainer {
    val leftJoystick: CommandJoystick = CommandJoystick(1)
    val rightJoystick: CommandJoystick = CommandJoystick(2)
    private val xboxController: CommandXboxController = CommandXboxController(2)

    val swerveSystem: SwerveSystem
    val trunkSystem = TrunkSystem(TrunkIOSim())

    val autonomousCommand: Command = Commands.run({})

    lateinit var teleopSwerveCommand: Command
    lateinit var teleopElevateCommand: Command
    lateinit var teleopRotateCommand: Command

//    val autoChooser: SendableChooser<Command> = AutoBuilder.buildAutoChooser()

    init {
        when (Constants.currentMode) {
            Constants.Mode.REAL -> {
                swerveSystem = SwerveSystem(
                        SwerveSystemIOReal(),
                        File(Filesystem.getDeployDirectory(), "yagsl_configs/slippy")
                )
            }

            Constants.Mode.SIM -> {
                // change these later
                swerveSystem = SwerveSystem(
                        SwerveSystemIOReal(),
                        File(Filesystem.getDeployDirectory(), "yagsl_configs/slippy")
                )
            }

            Constants.Mode.REPLAY -> {
                // change these later
                swerveSystem = SwerveSystem(
                        SwerveSystemIOReal(),
                        File(Filesystem.getDeployDirectory(), "yagsl_configs/slippy")
                )
            }
        }
//        SmartDashboard.putData("Auto Chooser", autoChooser)

        teleopSwerveCommand = TeleopSwerveDriveCommand()
        teleopSwerveCommand.schedule()

        configureButtonBindings()
    }

    private fun configureButtonBindings() {
        teleopElevateCommand = Commands.run({
            trunkSystem.elevate(-xboxController.leftY)
        })
        teleopRotateCommand = Commands.run({
            trunkSystem.rotate(-xboxController.rightY)
        }
        )
        xboxController.a().onTrue(Commands.runOnce({
            trunkSystem.intake()
        }))
        xboxController.y().onTrue(Commands.runOnce({
            trunkSystem.goToAmp()
        }))
        xboxController.x().onTrue(Commands.runOnce({
            trunkSystem.goManual()
        }))
        xboxController.b().onTrue(Commands.runOnce({
            trunkSystem.io.setZeroRotation()
        }))
    }
}