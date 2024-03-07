package frc.robot

import edu.wpi.first.math.VecBuilder
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.robot.commands.automatic.AutoClimbCommand
import frc.robot.util.Telemetry
import frc.robot.constants.LimelightConstants
import org.littletonrobotics.junction.LoggedRobot


class Robot : LoggedRobot() {
    private lateinit var m_autonomousCommand: Command

    private var lastRobotAction: RobotAction = RobotContainer.stateMachine.robotAction
    private var lastShootPosition: ShootPosition = RobotContainer.stateMachine.shootPosition

    private val autoClimbCommand: AutoClimbCommand = AutoClimbCommand()
    override fun robotInit() {
        SmartDashboard.putBoolean("arm motors free?", false)

        RobotContainer.swerveSystem.driveTrain.getDaqThread().setThreadPriority(99);
    }

    override fun robotPeriodic() {
        RobotContainer.swerveSystem.logger.telemeterize(RobotContainer.swerveSystem.driveTrain.state)

        SmartDashboard.putBoolean("Is trunk ready?", RobotContainer.stateMachine.trunkReady)

        CommandScheduler.getInstance().run()
        RobotContainer.stateMachine.logStates()

        if (RobotContainer.stateMachine.trunkState == TrunkState.MANUAL) {
            RobotContainer.trunkSystem.elevate(-RobotContainer.xboxController.leftY)
            RobotContainer.trunkSystem.rotate(-RobotContainer.xboxController.rightY * .1)
        } else {
//            if (!RobotContainer.trunkSystem.isMoving) {
//                RobotContainer.trunkSystem.io.setDesiredRotation(RobotContainer.trunkSystem.io.getRotation() + (-RobotContainer.xboxController.rightY * .1))
//            }
        }

        val armMotorsFree = SmartDashboard.getBoolean("arm motors free?", false)

        if (armMotorsFree && !DriverStation.isTeleopEnabled() && !DriverStation.isTestEnabled() && !DriverStation.isAutonomousEnabled()) {
            RobotContainer.trunkSystem.freeMotors()
        }
        else if (!armMotorsFree && !DriverStation.isTeleopEnabled() && !DriverStation.isTestEnabled() && !DriverStation.isAutonomousEnabled()){
            RobotContainer.trunkSystem.brakeMotors()
        }

        if (!DriverStation.isDisabled()) {
            RobotContainer.visionSystem.updateOdometry(2, true)
        }
        else {
            RobotContainer.visionSystem.updateOdometry(1, false)
        }
        
        Telemetry.putBoolean("shooter ready", RobotContainer.cannonSystem.shooterReady(), RobotContainer.telemetry.cannonTelemetry)
        Telemetry.putString("note state", RobotContainer.stateMachine.noteState.name, RobotContainer.telemetry.cannonTelemetry)
        Telemetry.putString("intake state", RobotContainer.stateMachine.intakeState.name, RobotContainer.telemetry.cannonTelemetry)
    }



    override fun disabledInit() {}

    override fun disabledPeriodic() {}

    override fun disabledExit() {}

    override fun autonomousInit() {
        RobotContainer.autonomousCommand.schedule()
    }

    override fun autonomousPeriodic() {}

    override fun autonomousExit() {}

    override fun teleopInit() {
        RobotContainer.cannonSystem.killShooter()
        RobotContainer.cannonSystem.killIntake()

        RobotContainer

        RobotContainer.autonomousCommand.cancel()
        RobotContainer.teleopSwerveCommand.schedule()
        RobotContainer.trunkSystem.calibrate()

//        RobotContainer.trunkSystem.STOP()


        SmartDashboard.putBoolean("Schedule Climb Command?", false)
        SmartDashboard.putBoolean("Pulldown Climb?", false)
    }

    override fun teleopPeriodic() {
        RobotContainer.stateMachine.TeleopAutomaticStateManagement()


        val scheduleClimbBool =SmartDashboard.getBoolean("Schedule Climb Command?", false)
        if (scheduleClimbBool && autoClimbCommand.isScheduled() == false) {
            autoClimbCommand.schedule()
        }
        else if (autoClimbCommand.isScheduled == true && scheduleClimbBool == false) {
            autoClimbCommand.cancel()
        }


        //        SmartDashboard.putNumber("JoyX", RobotContainer.rightJoystick.x)
        //        SmartDashboard.putNumber("JoyY", RobotContainer.rightJoystick.y)
        //        SmartDashboard.putNumber("JoyTwist", RobotContainer.rightJoystick.twist)
        //        val desiredState = SwerveModuleState(0.0, Rotation2d(0.0, 0.0))
        //        if(RobotContainer.xboxController.b().asBoolean)
        //            RobotContainer.trunkSystem.goManual()
        //        if(RobotContainer.xboxController.x().asBoolean)
        //            RobotContainer.trunkSystem.STOP()
        //        if(RobotContainer.xboxController.a().asBoolean)
        //            RobotContainer.trunkSystem.calibrate()
        //        RobotContainer.trunkSystem.elevate(-RobotContainer.xboxController.leftY)
        //        RobotContainer.trunkSystem.rotate(-RobotContainer.xboxController.rightY*.3)

    }

    override fun teleopExit() {}

    override fun testInit() {
        CommandScheduler.getInstance().cancelAll()
    }

    override fun testPeriodic() {
        //        RobotContainer.swerveSystem.drive(Translation2d(0.25, 0.0), 0.0, true)
        //        val calibrator = ShooterCalibrator("/u/shooter_calibrator/test1.csv");
        //        calibrator.writeOut(shots)
        //        val readShots = calibrator.readCsv();
        //        readShots.forEach {
        //            println(it.toCSV())
        //        }
    }

    override fun testExit() {}

    override fun simulationPeriodic() {}
}
