package frc.robot.subsystems.trunk

import MiscCalculations
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.util.Color8Bit
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.TrunkPosition
import frc.robot.TrunkState
import frc.robot.constants.TrunkConstants
import frc.robot.util.visualization.Mechanism2d
import frc.robot.util.visualization.MechanismLigament2d
import org.littletonrobotics.junction.Logger
import java.util.TimeZone
import kotlin.math.max
import kotlin.math.sqrt




class TrunkSystem(val io: TrunkIO) : SubsystemBase() {

    var currentState = TrunkState.CALIBRATING
    var targetPose = TrunkPosition.STOW
        set(value) =
            if (!this.isMoving) {
                field = value
            } else {
                field = field
            }


    private var prevTargetPose = TrunkPosition.SPEAKER

    var currentPosition: Double = 0.0
    var currentRotation: Double = io.getRotation()

    var isRotationSafe = false
    var hasElevatorMoved = false

    var isMoving = false

    var bottomisSet = false
    var bottomPosition = 0.0
    var topPosition = 0.0

    var rotationSetPoint = TrunkConstants.TARGET_SAFE_ANGLE
    var positionSetPoint = TrunkConstants.STOW_POSITION

    var shootingAngle = TrunkConstants.TARGET_SAFE_ANGLE

    val superstructureMechanism = Mechanism2d(TrunkConstants.TOP_BREAK_BEAM_POSITION * TrunkConstants.d2x + 1.0, TrunkConstants.TOP_BREAK_BEAM_POSITION * TrunkConstants.d2y + 1.0)
    val elevatorMechanismRoot = superstructureMechanism.getRoot("Elevator Root", 0.25, 0.25)
    val trunkMechanismRoot = superstructureMechanism.getRoot("Trunk Root", currentPosition * TrunkConstants.d2x + .25, currentPosition * TrunkConstants.d2y + .25)
    val trunkMechanism = trunkMechanismRoot.append(MechanismLigament2d("Trunk", -.25, currentRotation, color = Color8Bit(0, 0, 255)))
    val crossbarRoot = superstructureMechanism.getRoot("Crossbar Root", (TrunkConstants.CROSSBAR_BOTTOM + .02) * TrunkConstants.d2x + .25, (TrunkConstants.CROSSBAR_BOTTOM - .02) * TrunkConstants.d2y + 0.25)


//    fun setTargetPose(targetPose: TrunkPosition) {
//        if (!this.isMoving) {
//            this.targetPose = targetPose
//        }
//    }


    init {
        SmartDashboard.getNumber("bottom soft limit", 100.0)
        SmartDashboard.getNumber("top soft limit", 180.0)
        elevatorMechanismRoot.append(MechanismLigament2d("Elevator", .8, TrunkConstants.ELEVATOR_ANGLE))
        crossbarRoot.append(MechanismLigament2d("Crossbar", TrunkConstants.CROSSBAR_TOP - TrunkConstants.CROSSBAR_BOTTOM - .25, TrunkConstants.ELEVATOR_ANGLE, color = Color8Bit(0, 255, 0)))
        io.setDesiredRotation(TrunkConstants.TARGET_SAFE_ANGLE)
//        SmartDashboard.putNumber("Position SetPoint", 0.15)
//        SmartDashboard.putNumber("Rotation SetPoint", 150.0)
    }

    fun goManual() {
        io.setElevatorSpeed(0.0)
        io.setRotationSpeed(0.0)
        io.setPID(false)
        currentState = TrunkState.MANUAL
        io.setPositionLimits(true)
    }

    fun calibrate() {
        io.setElevatorSpeed(0.0)
        io.setRotationSpeed(0.0)
        io.setPID(false)
        bottomisSet = false
        currentState = TrunkState.CALIBRATING
        io.setElevatorSpeed(0.1)
        io.setPositionLimits(false)
    }

    fun rotate(speed: Double) {
        if (currentState == TrunkState.MANUAL) {
            io.setRotationSpeed(speed)
            SmartDashboard.putNumber("rotation percent output", speed)
        }
    }

    fun elevate(speed: Double) {
        if (currentState == TrunkState.MANUAL) {
            io.setElevatorSpeed(speed)
        }
    }
    fun goToCustom() {
        io.setElevatorSpeed(0.0)
        io.setPID(true)
        currentState = TrunkState.CUSTOM
        io.setPositionLimits(true)
    }

    override fun periodic() {
        SmartDashboard.putBoolean("is rotation safe?", isRotationSafe)
        SmartDashboard.putBoolean("has elevator moved?", hasElevatorMoved)
        SmartDashboard.putNumber("Angle val", io.getRotation())
        SmartDashboard.putNumber("position val", io.getPosition())
        SmartDashboard.putNumber("target position", targetPose.position)
        SmartDashboard.putNumber("target angle", targetPose.angle)
        SmartDashboard.putString("prevTargetPosition name", prevTargetPose.name)
        SmartDashboard.putString("targetPosition name", targetPose.name)

        if (currentState == TrunkState.CUSTOM) {
//            io.setDesiredPosition(TrunkConstants.STOW_POSITION)
            //Changed position
            if (targetPose != prevTargetPose) {
                io.setPID(true)
                isMoving = true
                //Not safe rotation
                if (!isRotationSafe && !hasElevatorMoved) {
                    io.setDesiredPosition(prevTargetPose.position)
                    io.setDesiredRotation(TrunkConstants.MIN_SAFE_ANGLE)
                } else if (isRotationSafe && !hasElevatorMoved) {
                    println("set the desired position to target position")
                    io.setDesiredRotation(TrunkConstants.MIN_SAFE_ANGLE)
                    io.setDesiredPosition(targetPose.position)
                }

                if (MiscCalculations.appxEqual(io.getPosition(), targetPose.position, .05)) {
                    hasElevatorMoved = true
                }

                //Elevator has moved
                if (hasElevatorMoved) {
                    io.setDesiredRotation(targetPose.angle)
                }

                //Elevator has moved and the angle is good
                if (hasElevatorMoved && MiscCalculations.appxEqual(io.getRotation(), targetPose.angle, 4.0)) {
                    prevTargetPose = targetPose
                    hasElevatorMoved = false
                    isMoving = false
                }
            }
            else {
//                io.setDesiredPosit
            }

            //Is rotation safe?
            if (io.getRotation() > TrunkConstants.MIN_SAFE_ANGLE || MiscCalculations.appxEqual(io.getRotation(), TrunkConstants.MIN_SAFE_ANGLE, 5.0)) {
                isRotationSafe = true
            }
        }
        else if (currentState == TrunkState.CALIBRATING) {
            doubleCalibratePeriodic()
        }
//        else if (currentState == TrunkState.STOP) {
//            stopPeriodic()
//        }
        io.periodic()
    }
    fun STOP() {
        io.setPID(false)
        currentState = TrunkState.STOP
        io.setElevatorSpeed(0.0)
        io.setRotationSpeed(0.0)
        io.setPositionLimits(true)
    }
    private fun doubleCalibratePeriodic() {
        val topLimit = io.atTopLimit()
        val bottomLimit = io.atBottomLimit()
        if (bottomLimit && !bottomisSet) {
            bottomisSet = true
            bottomPosition = io.getPosition()
        }
        if (topLimit) {
            topPosition = io.getPosition()
            io.setElevatorSpeed(0.0)
            io.setZeroPosition(true)
            STOP()
        }
    }
}
