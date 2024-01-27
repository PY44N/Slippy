package frc.robot

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.robot.constants.DriveConstants
import org.littletonrobotics.junction.LoggedRobot
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.NT4Publisher
import kotlin.math.abs


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
class Robot : LoggedRobot() {

    //    private val canCoder: CANcoder = CANcoder(1)
    private var autonomousCommand: Command? = null

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    override fun robotInit() {
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME)
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE)
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA)
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE)
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH)

        when (BuildConstants.DIRTY) {
            0 -> Logger.recordMetadata("GitDirty", "All changes committed")
            1 -> Logger.recordMetadata("GitDirty", "Uncomitted changes")
            else -> Logger.recordMetadata("GitDirty", "Unknown")
        }

        when (Constants.currentMode) {
            Constants.Mode.REAL -> {
                // Running on a real robot, log to a USB stick ("/U/logs")
//                Logger.addDataReceiver(WPILOGWriter())
                Logger.addDataReceiver(NT4Publisher())
            }

            Constants.Mode.SIM -> {
                // Running a physics simulator, log to NT
                Logger.addDataReceiver(NT4Publisher())
            }

            Constants.Mode.REPLAY -> {
                // Replaying a log, set up replay source
                setUseTiming(false) // Run as fast as possible
//                val logPath = LogFileUtil.findReplayLog()
//                Logger.setReplaySource(WPILOGReader(logPath))
//                Logger.addDataReceiver(WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")))
            }
        }

        Logger.start()

//        RobotContainer.speedUp += 1

//        val toApply = CANcoderConfiguration()


        /* User can change the configs if they want, or leave it empty for factory-default */
//        canCoder.getConfigurator().apply(toApply)
    }


    /**
     * This function is called every robot packet, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     *
     * This runs after the mode specific periodic functions, but before
     * LiveWindow and SmartDashboard integrated updating.
     */

    override fun robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run()
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    override fun disabledInit() {}

    /**
     * This function is called periodically when disabled.
     */
    override fun disabledPeriodic() {}

    /**
     * This autonomous runs the autonomous command selected by your [RobotContainer] class.
     */
    override fun autonomousInit() {
        // Schedule the autonomous command (example)
        // Note the Kotlin safe-call(?.), this ensures autonomousCommand is not null before scheduling it
    }

    /**
     * This function is called periodically during autonomous.
     */
    override fun autonomousPeriodic() {}

    /**
     * This function is called once when teleop is enabled.
     */
    override fun teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        // Note the Kotlin safe-call(?.), this ensures autonomousCommand is not null before cancelling it
        autonomousCommand?.cancel()

    }

    /**
     * This function is called periodically during operator control.
     */
    override fun teleopPeriodic() {
        SmartDashboard.putNumber("JoyX", RobotContainer.rightJoystick.x)
        SmartDashboard.putNumber("JoyY", RobotContainer.rightJoystick.y)
        SmartDashboard.putNumber("JoyTwist", RobotContainer.rightJoystick.twist)

        RobotContainer.swerveSystem.drive(
            Translation2d(
                (if (abs(RobotContainer.rightJoystick.x) > 0.15) {
                    val inSpeed =
                        if (RobotContainer.rightJoystick.x < 0.0) RobotContainer.rightJoystick.x + .15 else RobotContainer.rightJoystick.x - .15
                    (inSpeed) * DriveConstants.MAX_SPEED
                } else 0.0),
                (if (abs(RobotContainer.rightJoystick.y) > 0.15) {
                    val inSpeed =
                        if (RobotContainer.rightJoystick.y < 0.0) RobotContainer.rightJoystick.y + .15 else RobotContainer.rightJoystick.y - .15
                    (-inSpeed) * DriveConstants.MAX_SPEED
                } else 0.0)
            ),
            (if (abs(RobotContainer.rightJoystick.twist) > 0.15) -RobotContainer.rightJoystick.twist * -1.0 else 0.0),
            true
        )


        val desiredState =
            SwerveModuleState(0.0, Rotation2d(0.0, 0.0))

//        RobotContainer.swerveSystem.swerveDrive.setModuleStates(arrayOf(desiredState, desiredState, desiredState, desiredState), true)

//        SmartDashboard.putNumber("CANNNN", canCoder.position.value);
    }

    /**
     * This function is called once when test mode is enabled.
     */
    override fun testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll()
//        RobotContainer.swerveSystem.drive(Translation2d(0.25, 0.0), 0.0, true)
    }

    /**
     * This function is called periodically during test mode.
     */
    override fun testPeriodic() {
    }
}
