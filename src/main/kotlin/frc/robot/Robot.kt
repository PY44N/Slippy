package frc.robot

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import org.littletonrobotics.junction.LoggedRobot
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.NT4Publisher

class Robot : LoggedRobot() {

    private var autonomousCommand: Command? = null

    override fun robotInit() {

//        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME)
//        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE)
//        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA)
//        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE)
//        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH)
//
//        when (BuildConstants.DIRTY) {
//            0 -> Logger.recordMetadata("GitDirty", "All changes committed")
//            1 -> Logger.recordMetadata("GitDirty", "Uncomitted changes")
//            else -> Logger.recordMetadata("GitDirty", "Unknown")
//        }

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
        /* User can change the configs if they want, or leave it empty for factory-default */
//        canCoder.getConfigurator().apply(toApply)
        RobotContainer
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

    override fun disabledInit() {}

    override fun disabledPeriodic() {}

    override fun autonomousInit() {
        // Schedule the autonomous command (example)
        // Note the Kotlin safe-call(?.), this ensures autonomousCommand is not null before scheduling it
        autonomousCommand?.schedule()
    }

    override fun autonomousPeriodic() {}

    override fun teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        // Note the Kotlin safe-call(?.), this ensures autonomousCommand is not null before cancelling it
        autonomousCommand?.cancel()
        RobotContainer.teleopSwerveCommand.schedule()
        RobotContainer.teleopElevateCommand.schedule()

    }

    override fun teleopPeriodic() {

//        RobotContainer.swerveSystem.swerveDrive.setModuleStates(arrayOf(desiredState, desiredState, desiredState, desiredState), true)

//        SmartDashboard.putNumber("CANNNN", canCoder.position.value);
    }

    override fun testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll()
//        RobotContainer.swerveSystem.drive(Translation2d(0.25, 0.0), 0.0, true)
//        val calibrator = ShooterCalibrator("/u/shooter_calibrator/test1.csv");
//        calibrator.writeOut(shots)
//        val readShots = calibrator.readCsv();
//        readShots.forEach {
//            println(it.toCSV())
//        }
    }

    override fun testPeriodic() {}
}
