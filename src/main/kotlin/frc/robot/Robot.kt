package frc.robot

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.robot.commands.automatic.AutoClimbCommand
import frc.robot.commands.trunk.CalibrateTrunk
import frc.robot.constants.CannonConstants
import frc.robot.constants.TargetingConstants
import frc.robot.constants.TrunkConstants
import frc.robot.util.AllianceFlip
import frc.robot.util.Telemetry
import cshcyberhawks.lib.visualiztion.Field2d
import org.littletonrobotics.junction.LoggedRobot

class Robot : LoggedRobot() {
    private lateinit var m_autonomousCommand: Command

    private var lastRobotAction: RobotAction = RobotContainer.stateMachine.robotAction
    private var lastShootPosition: ShootPosition = RobotContainer.stateMachine.shootPosition

    private val autoClimbCommand: AutoClimbCommand = AutoClimbCommand()

    private var calibrateTrunkAuto: CalibrateTrunk = CalibrateTrunk()

    val robotPosePublisher = NetworkTableInstance.getDefault().getStructTopic("Robot Pose", Pose2d.struct).publish();
    val field2d = Field2d()

//    lateinit var autoChooser: SendableChooser<Command>

    override

    fun robotInit() {

        SmartDashboard.putNumber("g", 11.0)

        SmartDashboard.putBoolean("use new limelights", false)

        SmartDashboard.putBoolean("arm motors free?", false)
        SmartDashboard.putNumber("shooter endpoint x", TargetingConstants.endpointX)
        SmartDashboard.putNumber("shooter endpoint z", TargetingConstants.endpointZ)
        SmartDashboard.putNumber("shooter height", TargetingConstants.shooterZ)
        SmartDashboard.putNumber("shooter velocity transfer multiplier", TargetingConstants.velocityMultiplier)
        SmartDashboard.putNumber("shooter lead time", TargetingConstants.leadTime)

        SmartDashboard.putNumber("Current Target Angle", 0.0)
//        RobotContainer.swerveSystem.driveTrain.getDaqThread().setThreadPriority(99);

        SmartDashboard.putNumber("shooter angle", 58.5)

        SmartDashboard.putBoolean("Shooter Ready & Aimed", false)

        SmartDashboard.putNumber("Amp Speed", CannonConstants.INNER_AMP_PERCENT)
        SmartDashboard.putNumber("Amp Angle", TrunkConstants.AMP_ANGLE)
        SmartDashboard.putNumber("Amp Position", TrunkConstants.AMP_POSITION)
        SmartDashboard.putNumber("Safe To Move Angle", TrunkConstants.SAFE_TO_MOVE_ANGLE)

        SmartDashboard.putData("Field", field2d);

//        autoChooser =
//            AutoBuilder.buildAutoChooser()
//
//        SmartDashboard.putData("Auto Chooser", autoChooser);

        RobotContainer
    }

    override fun robotPeriodic() {

        SmartDashboard.putBoolean("Is trunk ready?", RobotContainer.trunkSystem.isAtPose)

        SmartDashboard.putString("Alliance", DriverStation.getAlliance().toString())

        TargetingConstants.endpointX = SmartDashboard.getNumber("shooter endpoint x", TargetingConstants.endpointX)
        TargetingConstants.endpointZ = SmartDashboard.getNumber("shooter endpoint z", TargetingConstants.endpointZ)
        TargetingConstants.shooterZ = SmartDashboard.getNumber("shooter height", TargetingConstants.shooterZ)
        TargetingConstants.velocityMultiplier =
            SmartDashboard.getNumber("shooter velocity transfer multiplier", TargetingConstants.velocityMultiplier)
        TargetingConstants.leadTime = SmartDashboard.getNumber("shooter lead time", TargetingConstants.leadTime)


        CommandScheduler.getInstance().run()
        RobotContainer.stateMachine.logStates()

        val armMotorsFree = SmartDashboard.getBoolean("arm motors free?", false)

        if (armMotorsFree && !DriverStation.isTeleopEnabled() && !DriverStation.isTestEnabled() && !DriverStation.isAutonomousEnabled()) {
            RobotContainer.trunkSystem.freeMotors()
        } else if (!armMotorsFree && !DriverStation.isTeleopEnabled() && !DriverStation.isTestEnabled() && !DriverStation.isAutonomousEnabled()) {
            RobotContainer.trunkSystem.brakeMotors()
        }

        if (!DriverStation.isDisabled() && !RobotContainer.stateMachine.limelightReset) {
//            RobotContainer.visionSystem.updateOdometry(1, false)
            RobotContainer.visionSystem.updateOdometry(1, true)

        } else {
//            RobotContainer.visionSystem.updateOdometry(1, false)
            RobotContainer.visionSystem.updateOdometryFromDisabled()
        }

        val shotSetup = RobotContainer.targetingSystem.getShotNoVelocity()
        SmartDashboard.putNumber("Mathed shooter angle", shotSetup.shooterAngle)
        SmartDashboard.putNumber("Mathed robot angle", shotSetup.robotAngle)
        SmartDashboard.putNumber("Current robot angle", RobotContainer.swerveSystem.getSwervePose().rotation.degrees)

        Telemetry.putBoolean(
            "shooter ready",
            RobotContainer.cannonSystem.shooterReady(),
            RobotContainer.telemetry.cannonTelemetry
        )
        Telemetry.putString(
            "note state",
            RobotContainer.stateMachine.noteState.name,
            RobotContainer.telemetry.cannonTelemetry
        )
        Telemetry.putString(
            "intake state",
            RobotContainer.stateMachine.intakeState.name,
            RobotContainer.telemetry.cannonTelemetry
        )


        RobotContainer.logger.telemeterize(RobotContainer.swerveSystem.swerveState())

        SmartDashboard.putNumber("Robot X", RobotContainer.swerveSystem.getSwervePose().x)
        SmartDashboard.putNumber("Robot Y", RobotContainer.swerveSystem.getSwervePose().y)
        SmartDashboard.putNumber("Robot Angle", RobotContainer.swerveSystem.getSwervePose().rotation.degrees)

//        val changedPosition = Field2d.toWPILIBFieldPosition(FieldPosition(fieldPosition.x, fieldPosition.y, gyro.getYaw()))
//        field2d.get().setRobotPose(changedPosition.x, changedPosition.y, Rotation2d(changedPosition.angleRadians))

        SmartDashboard.putString("Current Trunk Command", RobotContainer.stateMachine.currentTrunkCommand.name)
        SmartDashboard.putNumber("Trunk Target Rotation", RobotContainer.trunkSystem.trunkDesiredRotation)
//        SmartDashboard.putNumber("Trunk Rotation", RobotContainer.trunkSystem.getThroughboreRotation())
//        SmartDashboard.putNumber("Trunk Position", RobotContainer.trunkSystem.getPosition())

        SmartDashboard.putString("Cannon State", RobotContainer.stateMachine.intakeState.name)

//        SmartDashboard.putNumber("Falcon Raw Rotation", RobotContainer.trunkSystem.io.getFalconRawRotation())
//        SmartDashboard.putNumber("TB Raw Rotation", RobotContainer.trunkSystem.io.getThroughBoreRawRotation())
//        SmartDashboard.putNumber(
//            "TB Rotation",
//            RobotContainer.trunkSystem.getThroughboreRotation()
//        )
//        SmartDashboard.putNumber(
//            "Falcon Rotation",
//            RobotContainer.trunkSystem.getFalconRotation()
//        )

        CannonConstants.INNER_AMP_PERCENT = SmartDashboard.getNumber("Amp Speed", CannonConstants.INNER_AMP_PERCENT)
        TrunkConstants.AMP_ANGLE = SmartDashboard.getNumber("Amp Angle", TrunkConstants.AMP_ANGLE)
        TrunkConstants.AMP_POSITION = SmartDashboard.getNumber("Amp Position", TrunkConstants.AMP_POSITION)
        TrunkPose.AMP.angle = TrunkConstants.AMP_ANGLE
        TrunkPose.AMP.position = TrunkConstants.AMP_POSITION
        IntakeState.AmpSpitting.innerPercent = CannonConstants.INNER_AMP_PERCENT
        IntakeState.AmpSpitting.outerPercent = CannonConstants.INNER_AMP_PERCENT

//        SmartDashboard.putBoolean("Stow Beam Break", RobotContainer.cannonSystem.io.getLoadedBeamBreak())
//        SmartDashboard.putBoolean("Intake Beam Break", RobotContainer.cannonSystem.io.getEntryBeamBreak())
//        SmartDashboard.putNumber("Left Shooter Vel", RobotContainer.cannonSystem.io.getLeftShooterTBVel())
//        SmartDashboard.putNumber("Right Shooter Vel", RobotContainer.cannonSystem.io.getRightShooterTBVel())

//        SmartDashboard.putNumber("Intake Position", RobotContainer.cannonSystem.io.getIntakePosition())

        SmartDashboard.putNumber(
            "Robot Pos X Flipped",
            AllianceFlip.apply(RobotContainer.swerveSystem.getSwervePose()).x
        )

//        SmartDashboard.putNumber("Climb Servo Angle", RobotContainer.trunkSystem.io.getServoAngle())

//        SmartDashboard.putBoolean("Trunk Stowed Beam Break", RobotContainer.trunkSystem.io.atStowLimit())

//        SmartDashboard.putNumber("Servo Angle", RobotContainer.trunkSystem.io.getServoAngle())

        robotPosePublisher.set(RobotContainer.swerveSystem.getSwervePose())
        field2d.robotPose = RobotContainer.swerveSystem.getSwervePose()
//        SmartDashboard.putBoolean("Top Beam Break", RobotContainer.trunkSystem.io.atTopLimit())

        TrunkConstants.SAFE_TO_MOVE_ANGLE =
            SmartDashboard.getNumber("Safe To Move Angle", TrunkConstants.SAFE_TO_MOVE_ANGLE)

    }

    override fun disabledInit() {}

    override fun disabledPeriodic() {}

    override fun disabledExit() {}

    override fun autonomousInit() {
//        RobotContainer.autonomousCommand.schedule()
//        RobotContainer.swerveSystem.zeroGyro()
//        DriveBackAuto().schedule()
//        RobotContainer.swerveSystem.driveTrain.getAutoPath("Source Side Shot").schedule()

//        RobotContainer.swerveSystem.driveTrain.getAutoPath("3 Note Close").schedule()
//        RobotContainer.swerveSystem.driveTrain.getAutoPath("Source Shoot and Taxi").schedule()
//        RobotContainer.stateMachine.currentTrunkCommand = calibrateTrunkAuto.andThen(RobotContainer.swerveSystem.driveTrain.getAutoPath("3 Note Close"))

//        calibrateTrunkAuto.schedule()

//        RobotContainer.swerveSystem.driveTrain.getAutoPath("Source Side 2 Note").schedule()
        RobotContainer.swerveSystem.setGyroRotationDegrees(RobotContainer.swerveSystem.getSwervePose().rotation.degrees)

//        autoChooser.selected.schedule()

//        RobotContainer.swerveSystem.getAutoPath("Amp Side 3 Piece").schedule()
//        RobotContainer.swerveSystem.getAutoPath("Source Side 3 Note Center").schedule()
//            RobotContainer.swerveSystem.getAutoPath("4 Note Close").schedule()
//        RobotContainer.swerveSystem.getAutoPath("Source Side 2 Note Center").schedule()

       // RobotContainer.swerveSystem.getAutoPath("Amp Side Sneak").schedule()
     //        RobotContainer.swerveSystem.getAutoPath("Amp Side Sneak").schedule()
        RobotContainer.swerveSystem.getAutoPath("Testing").schedule()

        //        RobotContainer.stateMachine.currentTrunkCommand.schedule()

    }

    override fun autonomousPeriodic() {

//        if (calibrateTrunkAuto.isFinished && !RobotContainer.swerveSystem.driveTrain.getAutoPath("3 Note Close").isScheduled()) {
////            RobotContainer.swerveSystem.driveTrain.getAutoPath("3 Note Close").schedule()
//            println("scheduling the auto path")
//        }
    }

    override fun autonomousExit() {}

    override fun teleopInit() {
//        RobotContainer.climbLatch.angle = 0.0 // tune before testing

//        RobotContainer.trunkSystem.brakeMotors()

        RobotContainer.swerveSystem.setGyroRotationDegrees(RobotContainer.swerveSystem.getSwervePose().rotation.degrees)

        RobotContainer.stateMachine.currentTrunkCommand = CalibrateTrunk()
        RobotContainer.stateMachine.currentTrunkCommand.schedule()

        RobotContainer.cannonSystem.killShooter()
        RobotContainer.cannonSystem.killIntake()


        RobotContainer.autonomousCommand.cancel()
        RobotContainer.teleopSwerveCommand.schedule()
//        RobotContainer.trunkSystem.calibrate()

//        RobotContainer.trunkSystem.STOP()

        SmartDashboard.putBoolean("Schedule Climb Command?", false)
        SmartDashboard.putBoolean("Pulldown Climb?", false)
    }

    override fun teleopPeriodic() {
        RobotContainer.stateMachine.TeleopAutomaticStateManagement()

        val scheduleClimbBool = SmartDashboard.getBoolean("Schedule Climb Command?", false)
        if (scheduleClimbBool && autoClimbCommand.isScheduled() == false) {
            autoClimbCommand.schedule()
        } else if (autoClimbCommand.isScheduled == true && scheduleClimbBool == false) {
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

//        println("Current trunk command: " + RobotContainer.stateMachine.currentTrunkCommand.name)
    }

    override fun teleopExit() {}

    override fun testInit() {
        CommandScheduler.getInstance().cancelAll()
    }

    override fun testPeriodic() {
//        RobotContainer.trunkSystem.io.directlySetPercentElevatorFollower(.1)

        RobotContainer.trunkSystem.io.directlySetPercentElevatorMaster(.1)
//        RobotContainer.targetingSystem.test()
    }

    override fun testExit() {}

    override fun simulationPeriodic() {}
}
