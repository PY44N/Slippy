package frc.robot.commands.automatic

import edu.wpi.first.math.MathUtil.clamp
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.DriveState
import frc.robot.NoteState
import frc.robot.RobotContainer
import frc.robot.ShooterState
import frc.robot.TrunkPose
import frc.robot.commands.cannon.AutoShootCommand
import frc.robot.commands.trunk.HoldPositionGoToAngleTrunk
import frc.robot.constants.DriveConstants
import frc.robot.constants.TrunkConstants
import frc.robot.util.ControllerUtil
import cshcyberhawks.lib.math.Timer

class AutoAimTwistAndShoot : Command() {
    val autoShoot: AutoShootCommand = AutoShootCommand()

    val trunkCommand: HoldPositionGoToAngleTrunk = HoldPositionGoToAngleTrunk(TrunkPose.SPEAKER)

    val twistPIDController: PIDController = PIDController(10.0, 0.0, 0.1)

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
        val shotSetup = RobotContainer.targetingSystem.getVelocityShot()

        //Handle the cannon aiming component
        val shooterAngle = clamp(shotSetup.shooterAngle, TrunkConstants.MIN_SHOOT_ANGLE, TrunkConstants.MAX_SHOOT_ANGLE)
//        SmartDashboard.putBoolean("shot is possible?", shooterAngle == shotSetup.shooterAngle)
//        RobotContainer.trunkSystem.setShootingAngle(shooterAngle)
        trunkCommand.desiredAngle = shooterAngle

        //Handle the twisting component
        val driveTwist = twistPIDController.calculate(
            RobotContainer.swerveSystem.getSwervePose().rotation.degrees,
            shotSetup.robotAngle
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
            Math.toRadians(driveTwist)
        ).execute()

        //Can we shoot?
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
