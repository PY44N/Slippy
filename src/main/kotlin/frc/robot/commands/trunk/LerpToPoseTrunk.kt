package frc.robot.commands.trunk

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.RobotContainer
import frc.robot.TrunkPose
import cshcyberhawks.lib.math.Timer

class LerpToPoseTrunk(val pose: TrunkPose, val LERP_TIME: Double = 3.0) : Command() {
    var startPosition = RobotContainer.trunkSystem.getPosition()
    var startRotation = RobotContainer.trunkSystem.getThroughboreRotation()

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
        startRotation = RobotContainer.trunkSystem.getThroughboreRotation()
        timer.reset()
        timer.start()
    }

    override fun execute() {
        currentTargetPosition = lerpPosition(timer.get())
        currentTargetRotation = lerpRotation(timer.get())

        RobotContainer.trunkSystem.setDesiredRotation(currentTargetRotation)
        RobotContainer.trunkSystem.setDesiredPosition(currentTargetPosition)

        val rotationVolts = RobotContainer.trunkSystem.calculateRotationOut(currentTargetRotation, true)

        RobotContainer.trunkSystem.io.setRotationVoltage(rotationVolts)

        val elevatorPercent = RobotContainer.trunkSystem.calculatePositionOut(currentTargetPosition)

        RobotContainer.trunkSystem.io.setElevatorSpeed(elevatorPercent)

        println("holding pose trunk")
    }

    override fun isFinished(): Boolean {
        return timer.hasElapsed(LERP_TIME)
    }
}