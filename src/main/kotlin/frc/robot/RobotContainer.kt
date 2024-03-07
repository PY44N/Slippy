package frc.robot

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandJoystick
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.commands.TeleopSwerveDriveCommand
import frc.robot.commands.automatic.AutoAimAndShoot
import frc.robot.commands.automatic.AutoAimAndShootFromPosition
import frc.robot.commands.cannon.AutoAmp
import frc.robot.commands.cannon.AutoIntake
import frc.robot.commands.cannon.AutoShootCommand
import frc.robot.constants.TunerConstants
import frc.robot.subsystems.VisionSystem
import frc.robot.subsystems.cannon.CannonIOReal
import frc.robot.subsystems.cannon.CannonSystem
import frc.robot.subsystems.swerve.SwerveSystem
import frc.robot.subsystems.trunk.TrunkIOReal
import frc.robot.subsystems.trunk.TrunkSystem
import frc.robot.util.TargetingSystem
import frc.robot.util.TelemetryToggles

object RobotContainer {
    val leftJoystick: CommandJoystick = CommandJoystick(0)
    val rightJoystick: CommandJoystick = CommandJoystick(1)
    val xboxController: CommandXboxController = CommandXboxController(2)

    val telemetry = TelemetryToggles()


    val trunkSystem = TrunkSystem(TrunkIOReal())

    val stateMachine: RobotStateMachine = RobotStateMachine()

    val cannonSystem: CannonSystem = CannonSystem(CannonIOReal())

    val autonomousCommand: Command = Commands.run({})

    var teleopSwerveCommand: Command = TeleopSwerveDriveCommand()


    val targetingSystem: TargetingSystem = TargetingSystem()

    val visionSystem: VisionSystem = VisionSystem()


//    val autoChooser: SendableChooser<Command> = AutoBuilder.buildAutoChooser()

    val autoStateManagementEnableButton: Boolean
        get() = SmartDashboard.getBoolean("Enable Automatic State Management", false)

    val robotActionSendable: SendableChooser<RobotAction> = SendableChooser<RobotAction>()
    val shootPositionSendable: SendableChooser<ShootPosition> = SendableChooser<ShootPosition>()
    val trunkPositionSendable: SendableChooser<TrunkPosition> = SendableChooser<TrunkPosition>()

    val swerveSystem: SwerveSystem = SwerveSystem()

    init {
        configureBindings()

        RobotAction.entries.forEach {
            robotActionSendable.addOption(it.name, it)
        }

        ShootPosition.entries.forEach {
            shootPositionSendable.addOption(it.name, it)
        }

        TrunkPosition.entries.forEach {
            trunkPositionSendable.addOption(it.name, it)
        }
    }

    private fun configureBindings() {
//        xboxController.a().toggleOnTrue(AutoIntake())
//        xboxController.x().toggleOnTrue(AutoShootCommand())
////        xboxController.x().onTrue(Commands.runOnce({
////            println("x button pressed")
////            cannonSystem.shoot()
////        }))
//        xboxController.b().onTrue(Commands.runOnce({
//            cannonSystem.killShooter()
//        }))
//        xboxController.y().toggleOnTrue(AutoAmp())
//        //MURDER...KILL IT ALL
//        xboxController.start().onTrue(Commands.runOnce({
//            cannonSystem.killShooter()
//            cannonSystem.killIntake()
//        }))

        rightJoystick.button(3).onTrue(Commands.runOnce({
            when (stateMachine.robotAction) {
                RobotAction.Speaker -> {
                    if (stateMachine.shootPosition == ShootPosition.AutoAim) {
                        AutoAimAndShoot()
                    } else {
                        AutoAimAndShootFromPosition(stateMachine.shootPosition.position)
                    }
                };
                RobotAction.Amp -> AutoAmp();
                RobotAction.SourceIntake -> TODO("Not yet implemented");
                RobotAction.FloorIntake -> AutoIntake()
                RobotAction.Trap -> TODO("Not yet implemented")
                //Does literally nothing
                RobotAction.Chill -> println("*Hits blunt* Yoooooooo sup bra (currently in chill mode)")
            }
        }))

        xboxController.a().onTrue(Commands.runOnce({
            stateMachine.targetTrunkPose = TrunkPosition.STOW
        }))
        xboxController.b().toggleOnTrue(AutoIntake())
        xboxController.x().onTrue(Commands.runOnce({
            trunkSystem.calibrate()
        }))
        xboxController.back().onTrue(Commands.runOnce({
            trunkSystem.STOP()
        }))
        xboxController.start().onTrue(Commands.runOnce({
            trunkSystem.goManual()
        }))
        xboxController.y().onTrue(Commands.runOnce({
            trunkSystem.goToCustom()
        }))
        xboxController.rightBumper().toggleOnTrue(AutoAimAndShoot())
        xboxController.leftBumper().toggleOnTrue(AutoAmp())
        xboxController.back().toggleOnTrue(AutoShootCommand())
    }

//    val autoChooser: SendableChooser<Command> = AutoBuilder.buildAutoChooser()
//        SmartDashboard.putData("Auto Chooser", autoChooser)

}

