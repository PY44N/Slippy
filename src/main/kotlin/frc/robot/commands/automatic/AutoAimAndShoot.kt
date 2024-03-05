package frc.robot.commands.automatic

import MiscCalculations
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.kinematics.SwerveDriveOdometry
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.*
import frc.robot.commands.cannon.AutoShootCommand
import frc.robot.constants.DriveConstants
import frc.robot.constants.TrunkConstants
import frc.robot.util.Math

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
        underStage = false
        val shotSetup = RobotContainer.targetingSystem.getShotNoVelocity(underStage)

        SmartDashboard.putNumber("Raw Shot Angle", shotSetup.shooterAngle)
        SmartDashboard.putNumber("robot shot angle", shotSetup.robotAngle)

        if (shotSetup.shooterAngle > TrunkConstants.MAX_SHOOT_ANGLE || shotSetup.shooterAngle < TrunkConstants.MIN_SHOOT_ANGLE) {
            SmartDashboard.putBoolean("Shot Good", false)

            shotSetup.shooterAngle = MathUtil.clamp(shotSetup.shooterAngle, TrunkConstants.MIN_SHOOT_ANGLE, TrunkConstants.MAX_SHOOT_ANGLE)
        } else {
            SmartDashboard.putBoolean("Shot Good", true)
        }

        SmartDashboard.putNumber("Shot Angle", shotSetup.shooterAngle)
        SmartDashboard.putNumber("Good Shot Angle?", Math.wrapAroundAngles(shotSetup.shooterAngle))

        //Handle the cannon aiming component
//        RobotContainer.trunkSystem.setShootingAngle(shotSetup.shooterAngle)

        //Can we shoot?
        if (RobotContainer.stateMachine.trunkReady && !autoShoot.isScheduled
        ) {
//            autoShoot.schedule()
        }
    }

    override fun isFinished(): Boolean {
        return (autoShoot.isFinished) || RobotContainer.stateMachine.noteState == NoteState.Empty
    }

    override fun end(interrupted: Boolean) {
        println("Shootyboi Done")
        RobotContainer.stateMachine.driveState = DriveState.Teleop
    }
}