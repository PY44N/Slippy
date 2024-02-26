package frc.robot.subsystems.trunk

import edu.wpi.first.wpilibj.Timer
import java.lang.Math.pow
import kotlin.math.pow

class TrunkIOSim : TrunkIO {
    var desiredTrunkRotation = 0.0
    var lastTrunkRotation = 0.0
    var trunkRotationTimer = Timer()

    var trunkRotation = 0.0

    var desiredTraversalPercentage = 0.0
    var lastTraversalPercentage = 0.0
    var traversalPercentageTimer = Timer()
    var traversalPercentage = 0.0

    var elevatorSpeed = 0.0
    var rotationSpeed = 0.0

    override fun getPosition(): Double = traversalPercentage

    override fun getRotation(): Double = trunkRotation

    override fun setDesiredPosition(position: Double) {
        elevatorSpeed = 0.0
        traversalPercentageTimer.reset()
        lastTraversalPercentage = desiredTraversalPercentage
        desiredTraversalPercentage = position
        traversalPercentageTimer.start()
    }

    override fun setDesiredRotation(angle: Double) {
        rotationSpeed = 0.0
        trunkRotationTimer.reset()
        lastTrunkRotation = desiredTrunkRotation
        desiredTrunkRotation = angle
        trunkRotationTimer.start()
    }

    override fun setElevatorSpeed(speed: Double) {
        elevatorSpeed = speed / 10
    }

    override fun setRotationSpeed(speed: Double) {
        rotationSpeed = speed * 5
    }

    override fun setZeroPosition() {}

    override fun setZeroRotation() {}

    override fun atTopLimit(): Boolean {
        return true
    }

    override fun calibrate() {}

    private fun easeInOutCubic(x: Double): Double {
        return if (x < 0.5) {
            4 * x * x * x
        } else {
            1 - (-2 * x + 2).pow(3.0) / 2
        }
    }

    override fun periodic() {
        trunkRotation += rotationSpeed
        traversalPercentage += elevatorSpeed

        if (trunkRotationTimer.get() < 1) {
            trunkRotation = (desiredTrunkRotation - lastTrunkRotation) * easeInOutCubic(trunkRotationTimer.get()) + lastTrunkRotation
        } else {
            trunkRotation = desiredTrunkRotation
        }

        if (traversalPercentageTimer.get() < 1) {
            traversalPercentage = (desiredTraversalPercentage - lastTraversalPercentage) * easeInOutCubic(traversalPercentageTimer.get()) + lastTraversalPercentage
        } else {
            traversalPercentage = desiredTraversalPercentage
        }
    }
}