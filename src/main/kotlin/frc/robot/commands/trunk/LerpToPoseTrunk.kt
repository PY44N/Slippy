package frc.robot.commands.trunk

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.RobotContainer
import frc.robot.TrunkPose
import frc.robot.util.Timer

class LerpToPoseTrunk(val pose: TrunkPose, val LERP_TIME: Double = 3.0) : Command() {
    var startPosition = RobotContainer.trunkSystem.getPosition()
    var startRotation = RobotContainer.trunkSystem.getRotation()

    var currentTargetPosition = startPosition
    var currentTargetRotation = startRotation

    val timer = Timer()

    fun lerpPosition(t: Double): Double {
        return startPosition + (pose.position - startPosition) * (t / LERP_TIME)
    }

    fun lerpRotation(t: Double): Double {
        return startRotation + (pose.angle - startRotation) * (t / LERP_TIME)
    }

    override fun initialize() {
        startPosition = RobotContainer.trunkSystem.getPosition()
        startRotation = RobotContainer.trunkSystem.getRotation()
        timer.reset()
        timer.start()
    }

    override fun execute() {
        currentTargetPosition = lerpPosition(timer.get())
        currentTargetRotation = lerpRotation(timer.get())

        val rotationVolts = RobotContainer.trunkSystem.calculateRotationOut(currentTargetRotation)

        RobotContainer.trunkSystem.io.setRotationVoltage(rotationVolts)

        val elevatorPercent = RobotContainer.trunkSystem.calculatePositionOut(currentTargetPosition)

        RobotContainer.trunkSystem.io.setElevatorSpeed(elevatorPercent)

        println("holding pose trunk")
    }

    override fun isFinished(): Boolean {
        return RobotContainer.trunkSystem.checkAtPose(
                pose.angle,
                pose.position
        )
    }
}