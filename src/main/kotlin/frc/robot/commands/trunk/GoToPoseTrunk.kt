package frc.robot.commands.trunk

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.RobotContainer
import frc.robot.TrunkPose
import frc.robot.constants.TrunkConstants

class GoToPoseTrunk(val desiredPose: TrunkPose) : Command() {

    var currentTargetAngle: Double = TrunkConstants.SAFE_TRAVEL_ANGLE
    var currentTargetPosition: Double = RobotContainer.trunkSystem.getPosition()

    val isAngleSafe: Boolean
        get() = RobotContainer.trunkSystem.getRotation() >= TrunkConstants.SAFE_TRAVEL_ANGLE


    override fun initialize() {
        RobotContainer.trunkSystem.isAtPose = false
        RobotContainer.trunkSystem.setDesiredRotation(currentTargetAngle)
    }


    override fun execute() {
        SmartDashboard.putNumber("Current Target Angle", currentTargetAngle)

        if (isAngleSafe) {
            currentTargetAngle = desiredPose.angle
            currentTargetPosition = desiredPose.position
            RobotContainer.trunkSystem.setDesiredRotation(currentTargetAngle)
        }

        val rotationVolts = RobotContainer.trunkSystem.calculateRotationOut(currentTargetAngle)
        println("rotation volts: " + rotationVolts)
        RobotContainer.trunkSystem.io.setRotationVoltage(rotationVolts)

        val elevatorPercent = RobotContainer.trunkSystem.calculatePositionOut(currentTargetPosition)
        RobotContainer.trunkSystem.io.setElevatorSpeed(elevatorPercent)
    }

    override fun isFinished(): Boolean {
        return RobotContainer.trunkSystem.checkAtPose(RobotContainer.trunkSystem.trunkDesiredRotation, RobotContainer.trunkSystem.elevatorPIDController.setpoint)
    }

    override fun end(interrupted: Boolean) {
        if (interrupted == false) {
            RobotContainer.trunkSystem.isAtPose = true
        }
        RobotContainer.stateMachine.currentTrunkCommand = HoldPoseTrunk(desiredPose)
    }
}