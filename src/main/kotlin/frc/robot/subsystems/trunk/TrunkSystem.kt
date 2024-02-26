package frc.robot.subsystems.trunk

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.util.Color8Bit
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.constants.TrunkConstants
import frc.robot.util.visualization.Mechanism2d
import frc.robot.util.visualization.MechanismLigament2d
import kotlin.math.sqrt

enum class TrunkPosition {
    AMP,
    SPEAKER,
    INTAKE,
    STOW,
    TRAP,
    CALIBRATE,
    MANUAL_CONTROL,
    CUSTOM_POS,
}

class TrunkSystem(val io: TrunkIO) : SubsystemBase() {
    var targetPosition = TrunkPosition.CALIBRATE

    var rotationSetPoint = TrunkConstants.TARGET_SAFE_ANGLE
    var positionSetPoint = TrunkConstants.SPEAKER_POSITION
    var shootingAngle = TrunkConstants.TARGET_SAFE_ANGLE

    var isDefinitelyAboveCrossbar = false

    var calibrationMoving = false

    val superstructureMechanism = Mechanism2d(1.0, 1.0)
    val elevatorMechanismRoot = superstructureMechanism.getRoot("Elevator Root", 0.0, 0.0)
    val trunkMechanismRoot = superstructureMechanism.getRoot("Trunk Root", io.getPosition(), io.getPosition())
    val trunkMechanism = trunkMechanismRoot.append(MechanismLigament2d("Trunk", -.25, io.getRotation(), color = Color8Bit(0, 0, 255)))


    init {
        elevatorMechanismRoot.append(MechanismLigament2d("Elevator", sqrt(2.0), 45.0))
    }

    fun setDesiredPosition(position: Double) {
        positionSetPoint = position
        io.setDesiredPosition(position)
    }

    private fun setDesiredRotation(angle: Double) {
        rotationSetPoint = angle
        io.setDesiredRotation(angle)
    }

    fun goManual() {
        targetPosition = TrunkPosition.MANUAL_CONTROL
    }

    fun rotate(speed: Double) {
        if (targetPosition == TrunkPosition.MANUAL_CONTROL) {
            io.setRotationSpeed(speed)
        }
    }

    fun elevate(speed: Double) {
        if (targetPosition == TrunkPosition.MANUAL_CONTROL) {
            io.setElevatorSpeed(speed)
        }
    }

    val keyboard = GenericHID(0)

    override fun periodic() {
        trunkMechanismRoot.setPosition(io.getPosition(), io.getPosition())
        trunkMechanism.angle = io.getRotation()

        SmartDashboard.putData("Trunk Mechanism", superstructureMechanism)
        SmartDashboard.putString("Trunk Position", targetPosition.name)

        SmartDashboard.putString("target position", targetPosition.name)

        // TODO: Remove for actual robot
        if (keyboard.getRawButton(1)) {
            targetPosition = TrunkPosition.STOW
        }
        if (keyboard.getRawButton(2)) {
            targetPosition = TrunkPosition.INTAKE
        }
        if (keyboard.getRawButton(3)) {
            targetPosition = TrunkPosition.SPEAKER
        }
        if (keyboard.getRawButton(4)) {
            targetPosition = TrunkPosition.AMP
        }

        when (targetPosition) {
            TrunkPosition.TRAP -> {
                goToTrapPose()
            }

            TrunkPosition.AMP -> {
                goToAmpPose()
            }

            TrunkPosition.INTAKE -> {
                goToIntakePose()
            }

            TrunkPosition.SPEAKER -> {
                goToSpeakerPose()
            }

            TrunkPosition.STOW -> {
                goToStowPose()
            }

            TrunkPosition.CALIBRATE -> {
                calibratePeriodic()
            }

            TrunkPosition.MANUAL_CONTROL -> {}
            TrunkPosition.CUSTOM_POS -> {
                customPositionPeriodic()
            }
        }
    }

    private fun calibratePeriodic() {
        //        if(rotationSetPoint != GUNConstants.TARGET_SAFE_ANGLE)
        //            setDesiredRotation(GUNConstants.TARGET_SAFE_ANGLE)
        //        else if(!calibrationMoving && getRotation() >= GUNConstants.MIN_SAFE_ANGLE) {
        //            setElevatorSpeed(.2)
        //            calibrationMoving = true
        //        }
        if (!calibrationMoving) {
            io.setElevatorSpeed(.2)
            calibrationMoving = true
        }
        if (io.atTopLimit()) {
            io.setElevatorSpeed(0.0)
            calibrationMoving = false
            targetPosition = TrunkPosition.MANUAL_CONTROL
            io.setZeroPosition()
        }
    }


    private fun goToAmpPose() {
        if (rotationSetPoint != TrunkConstants.AMP_ANGLE)
            setDesiredRotation(TrunkConstants.AMP_ANGLE)
        if (positionSetPoint != TrunkConstants.AMP_POSITION) {
            if (isDefinitelyAboveCrossbar || io.getPosition() > TrunkConstants.CROSSBAR_TOP) {
                isDefinitelyAboveCrossbar = true
                setDesiredPosition(TrunkConstants.AMP_POSITION)
            } else if (io.getRotation() > TrunkConstants.MIN_SAFE_ANGLE)
                setDesiredPosition(TrunkConstants.AMP_POSITION)
        }
    }

    fun calibrate() {
        io.setElevatorSpeed(0.0)
        io.calibrate()
        targetPosition = TrunkPosition.CALIBRATE
    }

    fun stow() {
        io.setElevatorSpeed(0.0)
        targetPosition = TrunkPosition.STOW
    }

    private fun goToSpeakerPose() {
        if (rotationSetPoint != shootingAngle) {
            if (isDefinitelyAboveCrossbar || io.getPosition() > TrunkConstants.CROSSBAR_TOP) {
                isDefinitelyAboveCrossbar = true
                setDesiredRotation(shootingAngle) // maybe no
            } else if (rotationSetPoint != TrunkConstants.TARGET_SAFE_ANGLE)
                setDesiredRotation(TrunkConstants.TARGET_SAFE_ANGLE)
        }
        if (positionSetPoint != TrunkConstants.SPEAKER_POSITION) {
            if (isDefinitelyAboveCrossbar || io.getRotation() > TrunkConstants.MIN_SAFE_ANGLE)
                setDesiredPosition(TrunkConstants.SPEAKER_POSITION)
            else
                setDesiredPosition(TrunkConstants.CROSSBAR_BOTTOM)
        }
    }

    private fun goToStowPose() {
        if (rotationSetPoint != TrunkConstants.STOW_ANGLE) {
            if (isDefinitelyAboveCrossbar || io.getPosition() > TrunkConstants.CROSSBAR_TOP) {
                isDefinitelyAboveCrossbar = true
                setDesiredRotation(TrunkConstants.STOW_ANGLE) // maybe no
            } else if (rotationSetPoint != TrunkConstants.TARGET_SAFE_ANGLE)
                setDesiredRotation(TrunkConstants.TARGET_SAFE_ANGLE)
        }
        if (positionSetPoint != TrunkConstants.STOW_POSITION) {
            if (isDefinitelyAboveCrossbar || io.getRotation() > TrunkConstants.MIN_SAFE_ANGLE)
                setDesiredPosition(TrunkConstants.STOW_POSITION)
            else
                setDesiredPosition(TrunkConstants.CROSSBAR_BOTTOM)
        }
    }

    private fun goToTrapPose() {

    }

    private fun goToIntakePose() {
        isDefinitelyAboveCrossbar = false
        if (rotationSetPoint != TrunkConstants.INTAKE_ANGLE) {
            if (io.getPosition() < TrunkConstants.CROSSBAR_BOTTOM)
                setDesiredRotation(TrunkConstants.INTAKE_ANGLE)
            else if (rotationSetPoint != TrunkConstants.TARGET_SAFE_ANGLE)
                setDesiredRotation(TrunkConstants.TARGET_SAFE_ANGLE)
        }
        if (positionSetPoint != TrunkConstants.INTAKE_POSITION) {
            if (io.getRotation() > TrunkConstants.MIN_SAFE_ANGLE)
                setDesiredPosition(TrunkConstants.INTAKE_POSITION)
            else if (positionSetPoint != TrunkConstants.CROSSBAR_TOP)
                setDesiredPosition(TrunkConstants.CROSSBAR_TOP)
        }
    }

    fun goToAmp() {
        targetPosition = TrunkPosition.AMP
    }

    fun ampScore() {
    }

    fun intake() {
        targetPosition = TrunkPosition.INTAKE
    }

    fun customPosition() {
        targetPosition = TrunkPosition.CUSTOM_POS
    }

    private fun customPositionPeriodic() {
        val position = SmartDashboard.getNumber("Position SetPoint", 0.0)
        val rotation = SmartDashboard.getNumber("Rotation SetPoint", 0.0)
        //        if(position != positionSetPoint)
        setDesiredPosition(position)
        //        if(rotation != rotationSetPoint)
        setDesiredRotation(rotation)
    }
}
