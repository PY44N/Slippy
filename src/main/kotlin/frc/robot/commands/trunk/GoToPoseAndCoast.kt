package frc.robot.commands.trunk

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.RobotContainer
import frc.robot.TrunkPose
import frc.robot.constants.TrunkConstants
import frc.robot.util.Timer

class GoToPoseAndCoast(val desiredPose: TrunkPose, val coastPos: Double) : Command() {
    var currentTargetAngle: Double = TrunkConstants.SAFE_TRAVEL_ANGLE
    var currentTargetPosition: Double = RobotContainer.trunkSystem.getPosition()

    val isAngleSafe: Boolean
        get() = RobotContainer.trunkSystem.getThroughboreRotation() >= TrunkConstants.SAFE_TO_MOVE_ANGLE

    val isPositionAlwaysSafe: Boolean
        get() = RobotContainer.trunkSystem.getPosition() >= TrunkConstants.SAFE_PIVOT_POSITION && desiredPose.position >= TrunkConstants.SAFE_PIVOT_POSITION

    var coasting = false

    var timer = Timer()


    override fun initialize() {
        RobotContainer.trunkSystem.brakeMotors()

        RobotContainer.trunkSystem.isAtPose = false
        RobotContainer.trunkSystem.setDesiredRotation(currentTargetAngle)

        timer.reset()
        timer.start()

        RobotContainer.trunkSystem.setDesiredPosition(currentTargetPosition)

        currentTargetPosition = RobotContainer.trunkSystem.getPosition()
        currentTargetAngle = TrunkConstants.SAFE_TRAVEL_ANGLE
        coasting = false
    }

    override fun execute() {
        SmartDashboard.putNumber("Current Target Angle", currentTargetAngle)


        if (isAngleSafe || isPositionAlwaysSafe) {
            currentTargetAngle = desiredPose.angle
            currentTargetPosition = desiredPose.position
            RobotContainer.trunkSystem.setDesiredRotation(currentTargetAngle)
            RobotContainer.trunkSystem.setDesiredPosition(currentTargetPosition)
        }


        //        print("is angle safe? $isAngleSafe")
        //        print("is position always safe? $isPositionAlwaysSafe")

        val elevatorPercent = RobotContainer.trunkSystem.calculatePositionOut(currentTargetPosition)
        RobotContainer.trunkSystem.io.setElevatorSpeed(elevatorPercent)
        //        RobotContainer.trunkSystem.io.setElevatorSpeed(0.0)


        if (RobotContainer.trunkSystem.getPosition() < coastPos && !coasting) {
            RobotContainer.trunkSystem.io.rotationBrake = false
            RobotContainer.trunkSystem.io.setRotationVoltage(0.0)
            println("Coasting")
            coasting = true
        }

        if (!coasting) {
            val rotationVolts = RobotContainer.trunkSystem.calculateRotationOut(currentTargetAngle)
            RobotContainer.trunkSystem.io.setRotationVoltage(rotationVolts)
        }

        //        RobotContainer.trunkSystem.io.setRotationVoltage(0.0)
    }

    override fun isFinished(): Boolean {
        return RobotContainer.trunkSystem.checkAtPose(
            currentTargetAngle,
            currentTargetPosition,
        )

    }

    override fun end(interrupted: Boolean) {
        if (!interrupted) {
            RobotContainer.trunkSystem.isAtPose = true
        }
        RobotContainer.trunkSystem.io.rotationBrake = true
        RobotContainer.trunkSystem.io.setRotationVoltage(0.0)
        RobotContainer.trunkSystem.io.setElevatorSpeed(0.0)


        SmartDashboard.putNumber("Go to pose and coast time: ", timer.get())
    }
}