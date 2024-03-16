package frc.robot.commands.automatic

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.RobotContainer
import frc.robot.TrunkPose
import frc.robot.commands.trunk.GoToPoseAndHoldTrunk

class AutoClimbCommand : Command() {
    val climbCommand = GoToPoseAndHoldTrunk(TrunkPose.CLIMB)

    override fun initialize() {
        RobotContainer.stateMachine.currentTrunkCommand = climbCommand
        RobotContainer.stateMachine.currentTrunkCommandLocked = true
    }

    override fun execute() {
        val speed = -RobotContainer.xboxController.rightTriggerAxis / 10.0
        climbCommand.goToPose.currentTargetPosition += speed
    }

    override fun end(interrupted: Boolean) {
        RobotContainer.stateMachine.currentTrunkCommandLocked = false
        SmartDashboard.putBoolean("Pulldown Climb?", false)
    }
}
