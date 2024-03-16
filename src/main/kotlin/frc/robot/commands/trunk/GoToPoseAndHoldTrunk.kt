package frc.robot.commands.trunk

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.RobotContainer
import frc.robot.TrunkPose

class GoToPoseAndHoldTrunk(val desiredPose: TrunkPose) : Command() {
    val goToPose = GoToPoseTrunk(desiredPose)

    override fun initialize() {
        goToPose.schedule()
    }

    override fun isFinished() = goToPose.isFinished

    override fun end(interrupted: Boolean) {
        if (!interrupted)
            RobotContainer.trunkSystem.isAtPose = true
        RobotContainer.stateMachine.currentTrunkCommand = HoldPoseTrunk(desiredPose)
        goToPose.cancel()
    }
}
