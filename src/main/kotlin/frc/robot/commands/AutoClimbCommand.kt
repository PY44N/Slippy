package frc.robot.commands.automatic

import MiscCalculations
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.robot.RobotContainer
import frc.robot.TrunkPose
import frc.robot.commands.trunk.GoToPoseAndHoldTrunk
import frc.robot.commands.trunk.GoToPoseTrunk
import frc.robot.commands.trunk.HoldPoseTrunk
import frc.robot.commands.trunk.LerpToPoseTrunk

class AutoClimbCommand : Command() {
    val holdCommand = HoldPoseTrunk(TrunkPose.CLIMB)
    val climbCommand = GoToPoseTrunk(TrunkPose.CLIMB).andThen(holdCommand)

    var climbed = false

    // Go to climb pos
    // Position to .22
    // Angle to 62

    // or
    // Position to .23
    // Angle to 78
    // Position to .31

    override fun initialize() {
        RobotContainer.stateMachine.currentTrunkCommand = climbCommand
        RobotContainer.stateMachine.currentTrunkCommandLocked = true
        RobotContainer.actuallyDoClimb = false
        climbed = false
    }

    override fun execute() {
        val positionSpeed = -MiscCalculations.calculateDeadzone(RobotContainer.xboxController.rightX, 0.1) / 1000.0
        val rotationSpeed = -MiscCalculations.calculateDeadzone(RobotContainer.xboxController.leftX, 0.1)
        //        climbCommand.goToPose.currentTargetPosition += speed
        holdCommand.currentTargetPosition += positionSpeed
        holdCommand.currentTargetRotation += rotationSpeed

        if (RobotContainer.actuallyDoClimb && !climbed) {
            RobotContainer.stateMachine.currentTrunkCommandLocked = false
            RobotContainer.stateMachine.currentTrunkCommand = LerpToPoseTrunk(TrunkPose.CLIMB_STAGE_1, 2.0)/*.andThen(ParallelRaceGroup(HoldPoseTrunk(TrunkPose.CLIMB_STAGE_1), WaitCommand(1.0)))*/.andThen(LerpToPoseTrunk(TrunkPose.CLIMB_STAGE_2, 5.0))/*.andThen(ParallelRaceGroup(HoldPoseTrunk(TrunkPose.CLIMB_STAGE_2), WaitCommand(1.0)))*/.andThen(LerpToPoseTrunk(TrunkPose.CLIMB_STAGE_FINAL, 2.0)).andThen(HoldPoseTrunk(TrunkPose.CLIMB_STAGE_FINAL))
            RobotContainer.stateMachine.currentTrunkCommandLocked = true


            climbed = true
        }
    }

    override fun end(interrupted: Boolean) {
        RobotContainer.stateMachine.currentTrunkCommandLocked = false
        RobotContainer.actuallyDoClimb = false
        SmartDashboard.putBoolean("Pulldown Climb?", false)
    }
}
