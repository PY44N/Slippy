package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.RobotContainer
import frc.robot.TrunkPosition

class ArmPIDCalibrationAngleCommand : Command() {

    override fun initialize() {
        RobotContainer.cannonSystem.killShooter()
        RobotContainer.stateMachine.targetTrunkPose = TrunkPosition.CalibrationAngle
        RobotContainer.trunkSystem.goToCustom()

    }

    override fun execute() {

    }

    override fun isFinished(): Boolean {
        return RobotContainer.stateMachine.trunkReady
    }

    override fun end(interrupted: Boolean) {
//        RobotContainer.stateMachine.targetTrunkPose = TrunkPosition.STOW
    }
}