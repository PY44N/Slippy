package frc.robot.commands.automatic

import edu.wpi.first.math.MathUtil.clamp
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.DriveState
import frc.robot.NoteState
import frc.robot.RobotContainer
import frc.robot.ShooterState
import frc.robot.TrunkPose
import frc.robot.commands.cannon.AutoShootCommand
import frc.robot.commands.trunk.GoToPoseAndHoldTrunk
import frc.robot.commands.trunk.HoldPositionGoToAngleTrunk
import frc.robot.constants.DriveConstants
import frc.robot.constants.TrunkConstants
import frc.robot.util.ControllerUtil
import frc.robot.util.Timer

class AutoAimDumbTwistAndShoot : Command() {
    val autoShoot: AutoShootCommand = AutoShootCommand()

    val trunkCommand: HoldPositionGoToAngleTrunk = HoldPositionGoToAngleTrunk(TrunkPose.SPEAKER)

    val twistPIDController =
        ProfiledPIDController(1.0, 0.0, 0.1, TrapezoidProfile.Constraints(540.0, 720.0))

    val timer = Timer()

    override fun initialize() {
        RobotContainer.stateMachine.driveState = DriveState.TranslationTeleop
        RobotContainer.stateMachine.shooterState = ShooterState.Shooting
        RobotContainer.stateMachine.currentTrunkCommand = trunkCommand;

        twistPIDController.enableContinuousInput(0.0, 360.0);

        timer.reset()
        timer.start()
    }

    override fun execute() {
        val shotSetup = RobotContainer.targetingSystem.getShotNoVelocity()

//        println("Shooter Angle: ${shotSetup.shooterAngle}")

        //Handle the cannon aiming component
        val shooterAngle = clamp(shotSetup.shooterAngle, TrunkConstants.MIN_SHOOT_ANGLE, TrunkConstants.MAX_SHOOT_ANGLE)
//        SmartDashboard.putBoolean("shot is possible?", shooterAngle == shotSetup.shooterAngle)
//        RobotContainer.trunkSystem.setShootingAngle(shooterAngle)
        trunkCommand.desiredAngle = shooterAngle

        //Handle the twisting component
        val driveTwistPIDOutRadians = twistPIDController.calculate(
            RobotContainer.swerveSystem.getSwervePose().rotation.radians,
            Math.toRadians(shotSetup.robotAngle)
        )

        val driveTranslation = ControllerUtil.calculateJoyTranslation(
            RobotContainer.rightJoystick.x, RobotContainer.rightJoystick.y,
            ControllerUtil.calculateJoyThrottle(RobotContainer.leftJoystick.throttle),
            DriveConstants.TELEOP_DEADZONE_X,
            DriveConstants.TELEOP_DEADZONE_Y
        )

        RobotContainer.swerveSystem.applyDriveRequest(
            driveTranslation.x,
            driveTranslation.y,
            driveTwistPIDOutRadians
        ).execute()

        //Can we shoot?
//        if (RobotContainer.stateMachine.trunkReady && MiscCalculations.appxEqual(
//                        twistPIDController.setpoint,
//                        shotSetup.robotAngle,
//                        1.0
//                ) && !autoShoot.isScheduled
//        ) {
//            autoShoot.schedule()
//        }
        if (timer.hasElapsed(1.5) && !autoShoot.isScheduled)
            autoShoot.schedule()
    }

    override fun isFinished() = autoShoot.isFinished || RobotContainer.stateMachine.noteState == NoteState.Empty

    override fun end(interrupted: Boolean) {
//        RobotContainer.stateMachine.currentTrunkCommand = GoToPoseAndHoldTrunk(TrunkPose.HIGH_STOW)
        RobotContainer.actuallyDoShoot = false
        RobotContainer.stateMachine.driveState = DriveState.Teleop
    }
}
