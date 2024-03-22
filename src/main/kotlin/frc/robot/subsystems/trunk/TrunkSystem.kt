package frc.robot.subsystems.trunk

import MiscCalculations
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.constants.TrunkConstants
import frc.robot.util.Timer

class TrunkSystem(val io: TrunkIO) : SubsystemBase() {

    val lowRotationPIDController: ProfiledPIDController = ProfiledPIDController(
        TrunkConstants.lowRotationKP,
        TrunkConstants.lowRotationKI,
        TrunkConstants.lowRotationKD,
        TrapezoidProfile.Constraints(TrunkConstants.lowRotationMaxVelo, TrunkConstants.lowRotationMaxAcceleration)
    )

    val highRotationPIDController: ProfiledPIDController = ProfiledPIDController(
        TrunkConstants.highRotationKP,
        TrunkConstants.highRotationKI,
        TrunkConstants.highRotationKD,
        TrapezoidProfile.Constraints(TrunkConstants.highRotationMaxVelo, TrunkConstants.highRotationMaxAcceleration)
    )

    //    val rotationPIDController = PIDController(TrunkConstants.rotationKP, TrunkConstants.rotationKI, TrunkConstants.rotationKD)
    val lowRotationFeedForward: ArmFeedforward = ArmFeedforward(
        TrunkConstants.lowRotationFFkS,
        TrunkConstants.lowRotationFFkG,
        TrunkConstants.lowRotationFFkV,
        TrunkConstants.lowRotationFFkA
    )

    val highRotationFeedForward: ArmFeedforward = ArmFeedforward(
        TrunkConstants.highRotationFFkS,
        TrunkConstants.highRotationFFkG,
        TrunkConstants.highRotationFFkV,
        TrunkConstants.highRotationFFkA
    )

    val climbRotationPIDController: ProfiledPIDController = ProfiledPIDController(
        TrunkConstants.climbRotationKP,
        TrunkConstants.climbRotationKI,
        TrunkConstants.climbRotationKD,
        TrapezoidProfile.Constraints(TrunkConstants.climbRotationMaxVelo, TrunkConstants.climbRotationMaxAcceleration)
    )

    var positionLocked = false
        set(value) {
            if (value)
                io.setServoAngle(0.0)
            else
                io.setServoAngle(90.0)
            field = value
        }

    val elevatorPIDController: PIDController =
        PIDController(TrunkConstants.positionKP, TrunkConstants.positionKI, TrunkConstants.positionKD)

    var trunkDesiredRotation = TrunkConstants.STOW_ANGLE

    var falconRotationOffset = 0.0
    var falconRotationZeroed = false

    init {
        SmartDashboard.putNumber("Trunk Target Position", TrunkConstants.TOP_BREAK_BEAM_POSITION)
        SmartDashboard.putData("High Profiled PID", highRotationPIDController)
        SmartDashboard.putData("Low Profiled PID", lowRotationPIDController)
        SmartDashboard.putData("Climb Profiled PID", climbRotationPIDController)
        SmartDashboard.putData("Position PID", elevatorPIDController)

        positionLocked = false
    }

    var isAtPose: Boolean = false

    var lastVelocity = 0.0

    val clock = Timer()

    var lastBrakeMode = true

    override fun periodic() {
        SmartDashboard.putNumber("Angle val", getThroughboreRotation())
        SmartDashboard.putNumber("position val", getPosition())

//        if (atDesiredRotation()) {
//            io.setFalconThroughBoreOffset()
//        }
        var time = clock.get()
        time = if (time > 0.0) time else 1.0
        val velocity = io.getPivotVelocity()
        val acceleration = (velocity - lastVelocity) / time

        SmartDashboard.putNumber("Pivot Velocity", velocity)
        SmartDashboard.putNumber("Pivot Acceleration", acceleration)

        lastVelocity = velocity
        clock.restart()

//        if (io.atTopLimit() && getPosition() < 0.35) {
//            io.setZeroPosition()
//        }

        if (lastBrakeMode != io.rotationBrake) {
            lowRotationPIDController.reset(getThroughboreRotation())

            lastBrakeMode = io.rotationBrake
        }

        if (io.rotationBrake && atDesiredRotation() && !falconRotationZeroed) {
            falconRotationOffset = getRawFalconRotation() - getThroughboreRotation()
            falconRotationZeroed = true
            println("Zeroed falcon rotation")
        }

        SmartDashboard.putNumber("Falcon Offset", falconRotationOffset)

        SmartDashboard.putBoolean("Falcon Rotation Zeroed", falconRotationZeroed)
        SmartDashboard.putBoolean("Trunk Rotation Brake", io.rotationBrake)
        SmartDashboard.putBoolean("Trunk At Desired Rotation", atDesiredRotation())

    }

    fun setDesiredRotation(desiredRot: Double) {
        lowRotationPIDController.goal = TrapezoidProfile.State(desiredRot, 0.0)
        highRotationPIDController.goal = TrapezoidProfile.State(desiredRot, 0.0)
        climbRotationPIDController.goal = TrapezoidProfile.State(desiredRot, 0.0)
        trunkDesiredRotation = desiredRot
    }

    fun calculateRotationOut(desiredRot: Double, climb: Boolean = false): Double {
        trunkDesiredRotation = desiredRot
        SmartDashboard.putNumber("SOMETHING STUPID Trunk PID Desired Rot", trunkDesiredRotation)
        SmartDashboard.putNumber("SOMETHING STUPID Trunk PID Rot", getFalconRotation())
        SmartDashboard.putNumber("SOMETHING STUPID Trunk PID TB Rot", getThroughboreRotation())
        val rotationPIDOut = if (climb) {
            climbRotationPIDController.calculate(getFalconRotation(), trunkDesiredRotation)
        } else if (trunkDesiredRotation > 100.0) {// || getThroughboreRotation() > 100.0) {
            highRotationPIDController.calculate(getFalconRotation(), trunkDesiredRotation)
        } else {
            lowRotationPIDController.calculate(getThroughboreRotation(), trunkDesiredRotation)
        }
        SmartDashboard.putNumber("Trunk Trapezoid Velocity Error", lowRotationPIDController.velocityError)

//        println("rotation PID out: " + rotationPIDOut)
        val rotationFFOut = if (trunkDesiredRotation > 100.0) {// || getThroughboreRotation() > 100.0) {
            highRotationFeedForward.calculate(Math.toRadians(trunkDesiredRotation - 90.0), 0.0)
        } else {
            lowRotationFeedForward.calculate(Math.toRadians(trunkDesiredRotation - 90.0), 0.0)
        }
        SmartDashboard.putNumber("SOMETHING STUPID Trunk Rotation PID", rotationPIDOut)
        SmartDashboard.putNumber("SOMETHING STUPID Trunk Rotation Error", highRotationPIDController.positionError)
        SmartDashboard.putNumber("SOMETHING STUPID Trunk Rotation FF", rotationFFOut)
        SmartDashboard.putNumber("SOMETHING STUPID Uncapped rotation voltage: ", rotationPIDOut + rotationFFOut)
        return MathUtil.clamp(
            rotationPIDOut
                    + rotationFFOut, TrunkConstants.MIN_ROT_VOLTS, TrunkConstants.MAX_ROT_VOLTS
        )
    }

//    fun calculateClimbRotationOut(desiredRot: Double): Double {
//        trunkDesiredRotation = desiredRot
//        SmartDashboard.putNumber("SOMETHING STUPID Trunk PID Desired Rot", trunkDesiredRotation)
//        SmartDashboard.putNumber("SOMETHING STUPID Trunk PID Rot", getFalconRotation())
//        SmartDashboard.putNumber("SOMETHING STUPID Trunk PID TB Rot", getThroughboreRotation())
//        val rotationPIDOut = highRotationPIDController.calculate(getFalconRotation(), trunkDesiredRotation)
//
//        SmartDashboard.putNumber("Trunk Trapezoid Velocity Error", climbRotationPIDController.velocityError)
//
//        //        println("rotation PID out: " + rotationPIDOut)
//        val rotationFFOut = lowRotationFeedForward.calculate(Math.toRadians(trunkDesiredRotation - 90.0), 0.0)
//
//        SmartDashboard.putNumber("SOMETHING STUPID Trunk Rotation PID", rotationPIDOut)
//        SmartDashboard.putNumber("SOMETHING STUPID Trunk Rotation Error", climbRotationPIDController.positionError)
//        SmartDashboard.putNumber("SOMETHING STUPID Trunk Rotation FF", rotationFFOut)
//        SmartDashboard.putNumber("SOMETHING STUPID Uncapped rotation voltage: ", rotationPIDOut + rotationFFOut)
//        return MathUtil.clamp(
//            rotationPIDOut
//                    + rotationFFOut, TrunkConstants.MIN_ROT_VOLTS, TrunkConstants.MAX_ROT_VOLTS
//        )
//    }

    fun calculatePositionOut(inputDesiredPosition: Double): Double {
        if (positionLocked) {
            return 0.0
        }
        SmartDashboard.putNumber("Trunk Target Position", inputDesiredPosition)
        val posPIDOut = elevatorPIDController.calculate(getPosition(), inputDesiredPosition)
        val posFF = TrunkConstants.positionFF
        return posPIDOut + posFF
    }

    fun checkAtPose(pivotAngle: Double, elevatorPosition: Double): Boolean {
        return MiscCalculations.appxEqual(
            pivotAngle,
            getThroughboreRotation(),
            TrunkConstants.ANGLE_DEADZONE
        ) && MiscCalculations.appxEqual(elevatorPosition, getPosition(), TrunkConstants.ELEVATOR_DEADZONE)
    }

    fun checkAtPosition(position: Double): Boolean {
        return MiscCalculations.appxEqual(position, getPosition(), TrunkConstants.ELEVATOR_DEADZONE)
    }

    fun atDesiredRotation(): Boolean {
        return MiscCalculations.appxEqual(trunkDesiredRotation, getThroughboreRotation(), TrunkConstants.ANGLE_DEADZONE)
    }

    fun getThroughboreRotation(): Double {
        return frc.robot.util.Math.wrapAroundAngles((-io.getThroughBoreRawRotation() * 360.0) - TrunkConstants.throughboreRotationOffset)
//        return frc.robot.util.Math.wrapAroundAngles(io.getFalconRawRotation() * 360.0 - TrunkConstants.rotationOffset)
    }

    fun getFalconRotation(): Double {
        return frc.robot.util.Math.wrapAroundAngles((io.getFalconRawRotation() * 360.0) - falconRotationOffset)
    }

    private fun getRawFalconRotation(): Double {
        return frc.robot.util.Math.wrapAroundAngles((io.getFalconRawRotation() * 360.0))
    }

    fun getPosition(): Double {
        return io.getPosition() * TrunkConstants.ELEVATOR2M
    }

    fun freeMotors() {
        io.rotationBrake = false
        io.positionBrake = false
    }

    fun brakeMotors() {
        io.rotationBrake = true
        io.positionBrake = true
    }
}
