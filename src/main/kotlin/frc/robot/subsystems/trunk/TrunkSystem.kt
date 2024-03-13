package frc.robot.subsystems.trunk

import MiscCalculations
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotContainer
import frc.robot.constants.TrunkConstants


class TrunkSystem(val io: TrunkIO) : SubsystemBase() {

    val rotationPIDController: ProfiledPIDController = ProfiledPIDController(TrunkConstants.rotationKP, TrunkConstants.rotationKI, TrunkConstants.rotationKD, TrapezoidProfile.Constraints(TrunkConstants.rotationMaxVelo, TrunkConstants.rotationMaxAcceleration))
    val rotationFeedForward: ArmFeedforward = ArmFeedforward(TrunkConstants.rotationFFkS, TrunkConstants.rotationFFkG, TrunkConstants.rotationFFkV, TrunkConstants.rotationFFkA)

    val elevatorPIDController: PIDController = PIDController(TrunkConstants.positionKP, TrunkConstants.positionKI, TrunkConstants.positionKD)

    init {
    }

    var isAtPose: Boolean = false

    override fun periodic() {
        SmartDashboard.putNumber("Angle val", getRotation())
    }

    fun setDesiredRotation(desiredRot: Double) {
        rotationPIDController.goal = TrapezoidProfile.State(desiredRot, 0.0)
    }

    fun calculateRotationOut(desiredRot: Double): Double {
        val rotationPIDOut = rotationPIDController.calculate(getRotation(), desiredRot)
//        println("rotation PID out: " + rotationPIDOut)
        val rotationFFOut = rotationFeedForward.calculate(Math.toRadians(desiredRot - 90), 0.0)
        return MathUtil.clamp(rotationPIDOut
                + rotationFFOut, TrunkConstants.MIN_ROT_VOLTS, TrunkConstants.MAX_ROT_VOLTS)
    }

    fun calculatePositionOut(desiredPosition: Double): Double {
        val posPIDOut = elevatorPIDController.calculate(getPosition(), desiredPosition)
        val posFF = TrunkConstants.positionFF
        return posPIDOut + posFF
    }


    fun checkAtPose(pivotAngle: Double, elevatorPosition: Double): Boolean {
        return MiscCalculations.appxEqual(pivotAngle, getRotation(), TrunkConstants.ANGLE_DEADZONE) && MiscCalculations.appxEqual(elevatorPosition, getPosition(), TrunkConstants.ELEVATOR_DEADZONE)
    }

    fun checkAtPosition(position: Double): Boolean {
        return MiscCalculations.appxEqual(position, getPosition(), TrunkConstants.ELEVATOR_DEADZONE)
    }

    fun getRotation(): Double {
        return frc.robot.util.Math.wrapAroundAngles((-io.getRawRotation() * 360.0) - TrunkConstants.rotationOffset)
    }

    fun getPosition(): Double {
        return io.getRawPosition() * TrunkConstants.ELEVATOR2M
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
