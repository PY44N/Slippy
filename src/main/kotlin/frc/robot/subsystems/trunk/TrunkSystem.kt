package frc.robot.subsystems.trunk

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.util.Color8Bit
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.constants.TrunkConstants
import frc.robot.util.visualization.Mechanism2d
import frc.robot.util.visualization.MechanismLigament2d
import kotlin.math.max
import kotlin.math.sqrt

enum class TrunkPosition(val angle: Double, val position: Double) {
    AMP(TrunkConstants.AMP_ANGLE, TrunkConstants.AMP_POSITION),
    SPEAKER(TrunkConstants.TARGET_SAFE_ANGLE, TrunkConstants.SPEAKER_POSITION),
    INTAKE(TrunkConstants.INTAKE_ANGLE, TrunkConstants.INTAKE_POSITION),
    STOW(TrunkConstants.STOW_ANGLE, TrunkConstants.STOW_POSITION),
    TRAP(TrunkConstants.TRAP_ANGLE, TrunkConstants.TRAP_POSITION),;
}

class TrunkSystem(val io: TrunkIO) : SubsystemBase() {

    var targetPose = TrunkPosition.STOW

    var currentPosition: Double = 0.0
    var currentRotation: Double = io.getRotation()

    var isCalibrating = true
    var isManual = false
    var isTraveling = false
    var isShooting = false

    var rotationSetPoint = TrunkConstants.TARGET_SAFE_ANGLE
    var positionSetPoint = TrunkConstants.STOW_POSITION

    var shootingAngle = TrunkConstants.TARGET_SAFE_ANGLE

    val superstructureMechanism = Mechanism2d(TrunkConstants.TOP_BREAK_BEAM_POSITION * TrunkConstants.d2x, TrunkConstants.TOP_BREAK_BEAM_POSITION * TrunkConstants.d2y)
    val elevatorMechanismRoot = superstructureMechanism.getRoot("Elevator Root", 0.0, 0.0)
    val trunkMechanismRoot = superstructureMechanism.getRoot("Trunk Root", currentPosition * TrunkConstants.d2x, currentPosition * TrunkConstants.d2y)
    val trunkMechanism = trunkMechanismRoot.append(MechanismLigament2d("Trunk", -.25, currentRotation, color = Color8Bit(0, 0, 255)))


    init {
        elevatorMechanismRoot.append(MechanismLigament2d("Elevator", sqrt(2.0), 45.0))
        io.setDesiredRotation(TrunkConstants.TARGET_SAFE_ANGLE)
    }

    private fun setDesiredPosition(position: Double) {
        if(positionSetPoint != position) {
            positionSetPoint = position
            io.setDesiredPosition(position)
        }
    }

    private fun setDesiredRotation(angle: Double) {
        if(rotationSetPoint != angle) {
            rotationSetPoint = angle
            io.setDesiredRotation(angle)
        }
    }

    fun goManual() {
        isCalibrating = false
        isTraveling = false
        isManual = true
    }

    fun calibrate() {
        isManual = false
        isCalibrating = true
        io.setElevatorSpeed(0.0)
        io.disablePositionLimits()
    }

    fun rotate(speed: Double) {
        if (isManual) {
            io.setRotationSpeed(speed)
        }
    }

    fun elevate(speed: Double) {
        if (isManual) {
            io.setElevatorSpeed(speed)
        }
    }

    val keyboard = GenericHID(0)

    override fun periodic() {
        currentPosition = io.getPosition()
        currentRotation = io.getRotation()

        trunkMechanismRoot.setPosition(currentPosition*TrunkConstants.d2x, currentPosition*TrunkConstants.d2y)
        trunkMechanism.angle = currentRotation

        SmartDashboard.putData("Trunk Mechanism", superstructureMechanism)

        SmartDashboard.putNumber("Trunk Position", currentPosition)
        SmartDashboard.putNumber("Trunk Rotation", currentRotation)
        SmartDashboard.putNumber("Desired Position", positionSetPoint)

        // TODO: Remove for actual robot
        if (keyboard.getRawButton(1)) {
            targetPose = TrunkPosition.STOW
        }
        if (keyboard.getRawButton(2)) {
            targetPose = TrunkPosition.INTAKE
        }
        if (keyboard.getRawButton(3)) {
            targetPose = TrunkPosition.SPEAKER
        }
        if (keyboard.getRawButton(4)) {
            targetPose = TrunkPosition.AMP
        }


        if(isCalibrating) {
            calibratePeriodic()
        } else {
            if(io.atTopLimit())
                io.setZeroPosition(top = true)
            else if(io.atBottomLimit())
                io.setZeroPosition(top = false)
            if(!isManual) {
                goToTargetPosePeriodic()
            }
        }
        io.periodic()
    }

    private fun goToTravelPeriodic() {
        if(currentRotation >= TrunkConstants.MIN_SAFE_ANGLE)
            isTraveling = true
        setDesiredRotation(max(TrunkConstants.TARGET_SAFE_ANGLE, targetPose.angle))
        io.setBottomRotationLimit(TrunkConstants.MIN_SAFE_ANGLE)
    }

    private fun goToTargetPosePeriodic() {
        val targetPoseIsAboveCrossbar = targetPose.position >= TrunkConstants.CROSSBAR_TOP
        val targetPoseIsBelowCrossbar = targetPose.position <= TrunkConstants.CROSSBAR_BOTTOM
        val isAboveCrossbar = currentPosition > TrunkConstants.CROSSBAR_TOP
        val isBelowCrossbar = currentPosition <= TrunkConstants.CROSSBAR_BOTTOM
        if(isAboveCrossbar && targetPoseIsAboveCrossbar || isBelowCrossbar && targetPoseIsBelowCrossbar) {
            isTraveling = false
            setDesiredRotation(if(isShooting) shootingAngle else targetPose.angle)
            setDesiredPosition(targetPose.position)
            if(isAboveCrossbar) {
                io.setTopPositionLimit(TrunkConstants.TOP_BREAK_BEAM_POSITION)
                io.setBottomPositionLimit(TrunkConstants.CROSSBAR_TOP)
                io.setBottomRotationLimit(TrunkConstants.MIN_ANGLE_ABOVE_CROSSBAR)
            } else {
                io.setTopPositionLimit(TrunkConstants.CROSSBAR_BOTTOM)
                io.setBottomPositionLimit(TrunkConstants.BOTTOM_BREAK_BEAM_POSITION)
                io.setBottomRotationLimit(TrunkConstants.MIN_ANGLE_BELOW_CROSSBAR)
            }
        }
        else if(!isTraveling)
            goToTravelPeriodic()
        else
            setDesiredPosition(targetPose.position)
    }

    private fun calibratePeriodic() {
        if(!isTraveling)
            goToTravelPeriodic()
        else
            io.setElevatorSpeed(.2)

        val topLimit = io.atTopLimit()
        val bottomLimit = io.atBottomLimit()
        if (topLimit || bottomLimit) {
            io.setElevatorSpeed(0.0)
            isCalibrating = false
            io.setZeroPosition(topLimit)
            io.setTopPositionLimit(TrunkConstants.TOP_BREAK_BEAM_POSITION)
        }
    }

    private fun customPositionPeriodic() {
        val position = SmartDashboard.getNumber("Position SetPoint", 0.0)
        val rotation = SmartDashboard.getNumber("Rotation SetPoint", 0.0)
        setDesiredPosition(position)
        setDesiredRotation(rotation)
    }
}
