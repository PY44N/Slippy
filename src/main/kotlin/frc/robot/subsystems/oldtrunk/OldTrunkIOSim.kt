package frc.robot.subsystems.oldtrunk

import frc.robot.constants.TrunkConstants
import cshcyberhawks.lib.math.Timer

class OldTrunkIOSim() : OldTrunkIO {
    private var rotation = TrunkConstants.STOW_ANGLE
    private var position = TrunkConstants.TOP_BREAK_BEAM_POSITION

    private var rotationVelocity = 0.0
    private var positionVelocity = 0.0

    private var lastLoopTime = Timer.getFPGATimestamp()

    override var positionBrake = true
    override var rotationBrake = true

    override fun getPosition(): Double = position

    override fun getThroughBoreRawRotation(): Double = rotation

    override fun getFalconRawRotation(): Double = rotation

    override fun setFalconThroughBoreOffset() {}

    override fun setElevatorSpeed(speed: Double) {
        println("Elevator Speed: $speed")
    }

    override fun getPivotVelocity(): Double {
        return rotationVelocity
    }

    override fun directlySetPercentElevatorFollower(percent: Double) {
        TODO("Not yet implemented")
    }

    override fun directlySetPercentElevatorMaster(percent: Double) {
        TODO("Not yet implemented")
    }

    override fun setZeroPosition(top: Boolean) {
//        TODO("Not yet implemented")
    }

    override fun setRotationVoltage(volts: Double) {
//        TODO("Not yet implemented")
    }

    override fun getElevatorVelocity(): Double {
//        TODO("Not yet implemented")
        return 0.0
    }

    override fun getElevatorMotorAccel(): Double {
//        TODO("Not yet implemented")
        return 0.0
    }

    override fun atStowLimit(): Boolean {
//        TODO("Not yet implemented")
        return false
    }

    override fun atTopLimit(): Boolean = position == TrunkConstants.TOP_BREAK_BEAM_POSITION

    override fun setServoAngle(angle: Double) {}

    override fun getServoAngle(): Double = 0.0

    override fun periodic() {
        val currentTime = Timer.getFPGATimestamp()
        val dt = currentTime - lastLoopTime



        lastLoopTime = currentTime
    }
}