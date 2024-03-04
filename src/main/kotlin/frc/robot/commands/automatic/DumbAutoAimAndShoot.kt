package frc.robot.commands.automatic

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.*
import frc.robot.commands.cannon.AutoShootCommand
import frc.robot.constants.DriveConstants
import frc.robot.util.TargetingSystem

class DumbAutoAimAndShoot: Command() {
    val autoShoot: AutoShootCommand = AutoShootCommand()

    val twistPIDController: PIDController = PIDController(0.1, 0.0, 0.0)

    var underStage: Boolean = false

    override fun initialize() {
        RobotContainer.stateMachine.shooterState = ShooterState.Shooting
        RobotContainer.stateMachine.driveState = DriveState.TranslationTeleop

        if (RobotContainer.stateMachine.trunkState != TrunkState.Speaker && RobotContainer.stateMachine.trunkState != TrunkState.SpeakerFromStage) {
            if (RobotContainer.stateMachine.currentRobotZone == GlobalZones.Stage) {
                RobotContainer.stateMachine.trunkState = TrunkState.SpeakerFromStage
                underStage = true
            }
            else {
                RobotContainer.stateMachine.trunkState = TrunkState.Speaker
                underStage = false
            }
        }

        twistPIDController.enableContinuousInput(0.0, 360.0);


    }

    override fun execute() {
        val desiredRot = RobotContainer.targetingSystem.getShotNoVelocity()

        val driveTwist = RobotContainer.swerveSystem.autoTwistController.calculateRotation(Rotation2d.fromDegrees(desiredRot))

        val driveTranslation = RobotContainer.swerveSystem.calculateJoyTranslation(
            RobotContainer.rightJoystick.x, RobotContainer.rightJoystick.y,
            RobotContainer.swerveSystem.calculateJoyThrottle(RobotContainer.leftJoystick.throttle),
            DriveConstants.TELEOP_DEADZONE_X,
            DriveConstants.TELEOP_DEADZONE_Y
        )

        RobotContainer.swerveSystem.drive(driveTranslation, driveTwist.degrees, true)

        if (RobotContainer.swerveSystem.autoTwistController.isAtDesired() && !autoShoot.isScheduled) {
            autoShoot.schedule()
        }
    }

    override fun isFinished(): Boolean {
        return (autoShoot.isFinished) || RobotContainer.stateMachine.noteState == NoteState.Empty
    }

    override fun end(interrupted: Boolean) {
        RobotContainer.stateMachine.driveState = DriveState.Teleop
    }
}