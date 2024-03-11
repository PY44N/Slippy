package frc.robot.commands.trunk

import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.RobotContainer
import frc.robot.TrunkPosition
import frc.robot.constants.TrunkConstants

class HoldPositionTrunk(val pos: TrunkPosition): Command() {

    override fun execute() {
        val rotationPIDOut = RobotContainer.trunkSystem.rotationPIDController.calculate(RobotContainer.trunkSystem.getRotation(), pos.angle)
        val rotationFFOut = RobotContainer.trunkSystem.rotationFeedForward.calculate(Math.toRadians(RobotContainer.trunkSystem.rotationPIDController.setpoint.position - 90), 0.0)

        val rotationVolts =  MathUtil.clamp(rotationPIDOut
                + rotationFFOut, -.5, 2.0)

        RobotContainer.trunkSystem.io.setRotationVoltage(rotationVolts)


        val posPIDOut = RobotContainer.trunkSystem.elevatorPIDController.calculate(RobotContainer.trunkSystem.getRotation(), pos.position)
        val posFF = TrunkConstants.positionFF

        RobotContainer.trunkSystem.io.setElevatorSpeed(posFF + posPIDOut)
    }

    override fun isFinished(): Boolean {
        return false
    }
}