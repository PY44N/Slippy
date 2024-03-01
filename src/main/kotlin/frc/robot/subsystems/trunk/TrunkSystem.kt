package frc.robot.subsystems.trunk

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.util.Color8Bit
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.constants.TrunkConstants
import frc.robot.util.visualization.Mechanism2d
import frc.robot.util.visualization.MechanismLigament2d
import org.littletonrobotics.junction.Logger
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

    private var targetPose = TrunkPosition.STOW

    var currentPosition: Double = 0.0
    var currentRotation: Double = io.getRotation()

    var isCalibrating = false
    var isManual = false
    var isTraveling = false
    var isShooting = false
    var stop = true

    var rotationSetPoint = TrunkConstants.TARGET_SAFE_ANGLE
    var positionSetPoint = TrunkConstants.STOW_POSITION

    var shootingAngle = TrunkConstants.TARGET_SAFE_ANGLE

    val superstructureMechanism = Mechanism2d(TrunkConstants.TOP_BREAK_BEAM_POSITION * TrunkConstants.d2x + 1.0, TrunkConstants.TOP_BREAK_BEAM_POSITION * TrunkConstants.d2y + 1.0)
    val elevatorMechanismRoot = superstructureMechanism.getRoot("Elevator Root", 0.25, 0.25)
    val trunkMechanismRoot = superstructureMechanism.getRoot("Trunk Root", currentPosition * TrunkConstants.d2x + .25, currentPosition * TrunkConstants.d2y + .25)
    val trunkMechanism = trunkMechanismRoot.append(MechanismLigament2d("Trunk", -.25, currentRotation, color = Color8Bit(0, 0, 255)))
    val crossbarRoot = superstructureMechanism.getRoot("Crossbar Root", (TrunkConstants.CROSSBAR_BOTTOM +.02)* TrunkConstants.d2x+.25, (TrunkConstants.CROSSBAR_BOTTOM-.02) * TrunkConstants.d2y + 0.25)


    init {
        elevatorMechanismRoot.append(MechanismLigament2d("Elevator", .8, TrunkConstants.ELEVATOR_ANGLE))
        crossbarRoot.append(MechanismLigament2d("Crossbar", TrunkConstants.CROSSBAR_TOP-TrunkConstants.CROSSBAR_BOTTOM - .25, TrunkConstants.ELEVATOR_ANGLE, color=Color8Bit(0,255,0)))
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
        stop = false
    }

    fun calibrate() {
        isManual = false
        isCalibrating = true
        stop = false
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

    fun setTargetPose(pose: TrunkPosition) {
        targetPose = pose
        isCalibrating = false
        isTraveling = false
        isShooting = pose == TrunkPosition.SPEAKER
        isManual = false
    }

    fun STOP() {
        stop = true
        isCalibrating = false
        isManual = false
    }

    val keyboard = GenericHID(0)

    override fun periodic() {
        currentPosition = io.getPosition()
        currentRotation = io.getRotation()

        trunkMechanismRoot.setPosition(currentPosition*TrunkConstants.d2x + .25, currentPosition*TrunkConstants.d2y + .25)
        trunkMechanism.angle = io.getRawRotation()

        SmartDashboard.putData("Trunk Mechanism", superstructureMechanism)

        SmartDashboard.putNumber("Trunk Position", currentPosition)
        SmartDashboard.putNumber("Trunk Rotation", currentRotation)
        SmartDashboard.putNumber("Desired Position", positionSetPoint)
        SmartDashboard.putNumber("Desired Rotation", rotationSetPoint)

        SmartDashboard.putString("setpose", targetPose.name)
        SmartDashboard.putBoolean("calibrating", isCalibrating)
        SmartDashboard.putBoolean("traveling", isTraveling)
        SmartDashboard.putBoolean("shooting", isShooting)
        SmartDashboard.putNumber("shooting angle", shootingAngle)
        SmartDashboard.putBoolean("manual", isManual)
        SmartDashboard.putBoolean("top limit", io.atTopLimit())
        SmartDashboard.putBoolean("bottom limit", io.atBottomLimit())

        // TODO: Remove for actual robot
        if (keyboard.getRawButton(1)) {
            setTargetPose(TrunkPosition.INTAKE)
        }
        if (keyboard.getRawButton(2)) {
            setTargetPose(TrunkPosition.STOW)
        }
        if (keyboard.getRawButton(3)) {
            setTargetPose(TrunkPosition.AMP)
        }
        if (keyboard.getRawButton(4)) {
            setTargetPose(TrunkPosition.TRAP)
        }

        if(stop) {
            io.setRotationSpeed(0.0)
            io.setElevatorSpeed(0.0)
        }
        else if(isCalibrating) {
            calibratePeriodic()
//        } else {
//            if(io.atTopLimit())
//                io.setZeroPosition(top = true)
//            else if(io.atBottomLimit())
//                io.setZeroPosition(top = false)
//            if(!isManual) {
//                goToTargetPosePeriodic()
//            }
        }
        io.periodic()
//        Logger.recordOutput("superstructureMechanism", superstructureMechanism
    }

    private fun goToTravelPeriodic() {
        if(currentRotation >= TrunkConstants.MIN_SAFE_ANGLE)
            isTraveling = true
        setDesiredRotation(max(TrunkConstants.TARGET_SAFE_ANGLE, targetPose.angle))
        io.setBottomRotationLimit(TrunkConstants.MIN_SAFE_ANGLE)
    }

    private fun goToTargetPosePeriodic() {
        var number = 0.0
        val targetPoseIsAboveCrossbar = targetPose.position >= TrunkConstants.CROSSBAR_TOP
        val targetPoseIsBelowCrossbar = targetPose.position <= TrunkConstants.CROSSBAR_BOTTOM
        val isAboveCrossbar = currentPosition > TrunkConstants.CROSSBAR_TOP
        val isBelowCrossbar = currentPosition <= TrunkConstants.CROSSBAR_BOTTOM
        if(isAboveCrossbar && targetPoseIsAboveCrossbar || isBelowCrossbar && targetPoseIsBelowCrossbar) {
            number = 1.0
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
        else if(!isTraveling) {
            goToTravelPeriodic()
            number = 2.0
        }
        else {
            setDesiredPosition(targetPose.position)
            number = 3.0
        }
        SmartDashboard.putNumber("asdfkjasdh", number)
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
