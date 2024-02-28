package frc.robot.subsystems.trunk

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
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

    var desiredElevatorSpeed = 0.0
    var desiredRotationSpeed = 0.0

    override fun getPosition(): Double = traversalPercentage

    override fun getRotation(): Double = trunkRotation

    override fun setDesiredPosition(position: Double) {
        desiredElevatorSpeed = 0.0
        traversalPercentageTimer.reset()
        lastTraversalPercentage = desiredTraversalPercentage
        desiredTraversalPercentage = position
        traversalPercentageTimer.start()
    }

    override fun setDesiredRotation(angle: Double) {
        desiredRotationSpeed = 0.0
        trunkRotationTimer.reset()
        lastTrunkRotation = desiredTrunkRotation
        desiredTrunkRotation = angle
        trunkRotationTimer.start()
    }

    override fun setElevatorSpeed(speed: Double) {
        desiredElevatorSpeed = speed / 10
    }

    override fun setRotationSpeed(speed: Double) {
        desiredRotationSpeed = speed * 5
    }

    override fun setZeroPosition(top: Boolean) {}

    override fun atTopLimit(): Boolean {
        return true
    }

    override fun atBottomLimit(): Boolean {
        return true
    }

    override fun disablePositionLimits() {}

    private fun easeInOutCubic(x: Double): Double {
        return if (x < 0.5) {
            4 * x * x * x
        } else {
            1 - (-2 * x + 2).pow(3.0) / 2
        }
    }
    override fun setTopPositionLimit(position: Double) {}
    override fun setBottomPositionLimit(position: Double) {}
    override fun setTopRotationLimit(rotation: Double) {}
    override fun setBottomRotationLimit(rotation: Double) {}

    override fun periodic() {
        trunkRotation += desiredRotationSpeed
        traversalPercentage += desiredElevatorSpeed
        SmartDashboard.putNumber("Traversal Percentage", traversalPercentage)
        SmartDashboard.putNumber("Traversal Setpoint", desiredTraversalPercentage)

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