package frc.robot.subsystems.trunk

import MiscCalculations
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
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


    fun calculateRotationOut(desiredRot: Double): Double {
        val rotationPIDOut = RobotContainer.trunkSystem.rotationPIDController.calculate(RobotContainer.trunkSystem.getRotation(), desiredRot)
        val rotationFFOut = RobotContainer.trunkSystem.rotationFeedForward.calculate(Math.toRadians(RobotContainer.trunkSystem.rotationPIDController.setpoint.position - 90), 0.0)
        return MathUtil.clamp(rotationPIDOut
                + rotationFFOut, -.5, 2.0)
    }

    fun calculatePositionOut(desiredPosition: Double): Double {
        val posPIDOut = RobotContainer.trunkSystem.elevatorPIDController.calculate(RobotContainer.trunkSystem.getRotation(), desiredPosition)
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
