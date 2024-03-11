package frc.robot.commands.trunk

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.RobotContainer
import frc.robot.TrunkPosition
import frc.robot.constants.TrunkConstants

class GoToPositionTrunk(val desiredPose: TrunkPosition): Command() {


    var currentTargetAngle: Double = TrunkConstants.SAFE_TRAVEL_ANGLE
    var currentTargetPose: Double = RobotContainer.trunkSystem.getPosition()

    val isAngleSafe: Boolean
            get() = RobotContainer.trunkSystem.getRotation() >= TrunkConstants.SAFE_TRAVEL_ANGLE


    override fun initialize() {

    }


    override fun execute() {
        if (isAngleSafe) {
            currentTargetAngle = desiredPose.angle
            currentTargetPose = desiredPose.position
        }


        val rotationPIDOut = RobotContainer.trunkSystem.rotationPIDController.calculate(RobotContainer.trunkSystem.getRotation(), currentTargetAngle)
        val rotationFFOut = RobotContainer.trunkSystem.rotationFeedForward.calculate(Math.toRadians(RobotContainer.trunkSystem.rotationPIDController.setpoint.position - 90), 0.0)

        val rotationVolts =  MathUtil.clamp(rotationPIDOut
                + rotationFFOut, -.5, 2.0)

        RobotContainer.trunkSystem.io.setRotationVoltage(rotationVolts)


        val posPIDOut = RobotContainer.trunkSystem.elevatorPIDController.calculate(RobotContainer.trunkSystem.getRotation(), currentTargetPose)
        val posFF = TrunkConstants.positionFF

        RobotContainer.trunkSystem.io.setElevatorSpeed(posFF + posPIDOut)
    }

    override fun isFinished(): Boolean {
        return RobotContainer.trunkSystem.isAtPosition(RobotContainer.trunkSystem.rotationPIDController.goal.position, RobotContainer.trunkSystem.elevatorPIDController.setpoint)
    }

    override fun end(interrupted: Boolean) {
        RobotContainer.trunkSystem.io.setElevatorSpeed(0.0)
        RobotContainer.trunkSystem.io.setRotationVoltage(0.0)
        RobotContainer.stateMachine.currentTrunkCommand = HoldPositionTrunk(desiredPose)
    }
}