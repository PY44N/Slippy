package frc.robot.commands.automatic


import MiscCalculations
import edu.wpi.first.math.MathUtil.clamp
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.*
import frc.robot.commands.cannon.AutoShootCommand
import frc.robot.constants.DriveConstants
import frc.robot.constants.TargetingConstants
import frc.robot.constants.TrunkConstants


class AutoAimAndShoot : Command() {
    val autoShoot: AutoShootCommand = AutoShootCommand()

    val waitForTwist: Boolean = true

    override fun initialize() {
        RobotContainer.stateMachine.shooterState = ShooterState.Shooting

        if (RobotContainer.stateMachine.targetTrunkPose != TrunkPosition.SPEAKER && RobotContainer.stateMachine.targetTrunkPose != TrunkPosition.SPEAKER_FROM_STAGE) {
            if (RobotContainer.stateMachine.currentRobotZone == GlobalZones.Stage) {
                RobotContainer.stateMachine.targetTrunkPose = TrunkPosition.SPEAKER_FROM_STAGE
            } else {
                RobotContainer.stateMachine.targetTrunkPose = TrunkPosition.SPEAKER
            }
        }
    }

    override fun execute() {
        val shotSetup = RobotContainer.targetingSystem.getShotNoVelocity()

        //Handle the cannon aiming component
        val shooterAngle = clamp(shotSetup.shooterAngle, TrunkConstants.MIN_SHOOT_ANGLE, TrunkConstants.MAX_SHOOT_ANGLE)
        RobotContainer.trunkSystem.setShootingAngle(shooterAngle)
        RobotContainer.trunkSystem.goToCustom()
        //Handle the cannon aiming component
        RobotContainer.trunkSystem.setShootingAngle(shooterAngle)

        SmartDashboard.putBoolean("Is shot angle possible?", shooterAngle == shotSetup.shooterAngle);


        if (waitForTwist) {
            val robotAngleGood: Boolean =  MiscCalculations.appxEqual(RobotContainer.swerveSystem.getSwervePose().rotation.degrees, shotSetup.robotAngle, TargetingConstants.ROBOT_ANGLE_DEADZONE)
            SmartDashboard.putBoolean("Shot aimed and ready?", shooterAngle == shotSetup.shooterAngle && robotAngleGood)
            if (RobotContainer.stateMachine.trunkReady && !autoShoot.isScheduled && robotAngleGood) {
                autoShoot.schedule()
            }
        }
        else {
            SmartDashboard.putBoolean("Shot aimed and ready?", RobotContainer.stateMachine.trunkReady)
            if (RobotContainer.stateMachine.trunkReady && !autoShoot.isScheduled) {
                autoShoot.schedule()
            }
        }
    }

    override fun isFinished(): Boolean {
        return (autoShoot.isFinished) || RobotContainer.stateMachine.noteState == NoteState.Empty
    }

    override fun end(interrupted: Boolean) {
        println("Shootyboi Done")
        RobotContainer.stateMachine.shooterState = ShooterState.Stopped
        RobotContainer.stateMachine.driveState = DriveState.Teleop
//        RobotContainer.stateMachine.targetTrunkPose = TrunkPosition.STOW
    }
}