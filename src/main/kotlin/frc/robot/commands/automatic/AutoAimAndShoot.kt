package frc.robot.commands.automatic

import MiscCalculations
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.*
import frc.robot.commands.cannon.AutoShootCommand
import frc.robot.constants.DriveConstants

class AutoAimAndShoot : Command() {
    val autoShoot: AutoShootCommand = AutoShootCommand()

    var underStage: Boolean = false

    override fun initialize() {
        RobotContainer.stateMachine.shooterState = ShooterState.Shooting

        if (RobotContainer.stateMachine.targetTrunkPose != TrunkPosition.SPEAKER && RobotContainer.stateMachine.targetTrunkPose != TrunkPosition.SPEAKER_FROM_STAGE) {
            if (RobotContainer.stateMachine.currentRobotZone == GlobalZones.Stage) {
                RobotContainer.stateMachine.targetTrunkPose = TrunkPosition.SPEAKER_FROM_STAGE
                underStage = true
            } else {
                RobotContainer.stateMachine.targetTrunkPose = TrunkPosition.SPEAKER
                underStage = false
            }
        }


    }

    override fun execute() {
        val shotSetup = RobotContainer.targetingSystem.getShotNoVelocity(underStage)

        //Handle the cannon aiming component
        RobotContainer.trunkSystem.setShootingAngle(shotSetup.shooterAngle)

        //Can we shoot?
        if (RobotContainer.stateMachine.trunkReady && !autoShoot.isScheduled
        ) {
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