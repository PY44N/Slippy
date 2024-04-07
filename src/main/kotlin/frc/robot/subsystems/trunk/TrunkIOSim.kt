package frc.robot.subsystems.trunk

import frc.robot.constants.TrunkConstants
import frc.robot.util.Timer

class TrunkIOSim() : TrunkIO {
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

    }

    override fun getPivotVelocity(): Double {
        return rotationVelocity
    }

    override fun setZeroPosition() {}

    override fun setRotationVoltage(volts: Double) {
        TODO("Not yet implemented")
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