package frc.robot.subsystems.trunk

import MiscCalculations
import com.revrobotics.CANSparkBase
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.controller.ElevatorFeedforward
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.util.Color8Bit
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotContainer
import frc.robot.TrunkPosition
import frc.robot.TrunkState
import frc.robot.constants.TrunkConstants
import frc.robot.util.Telemetry
import frc.robot.util.visualization.Mechanism2d
import frc.robot.util.visualization.MechanismLigament2d
import kotlin.math.abs


class TrunkSystem(val io: TrunkIO) : SubsystemBase() {

    val rotationPIDController: ProfiledPIDController = ProfiledPIDController(0.0, 0.0, 0.0, TrapezoidProfile.Constraints(0.0, 0.0))
    val rotationFeedForward: ArmFeedforward = ArmFeedforward(0.0, 0.0, 0.0)

    val elevatorPIDController: PIDController = PIDController(0.0, 0.0, 0.0)


    init {

    }



    fun isAtPosition(pivotAngle: Double, elevatorPosition: Double): Boolean {
        return MiscCalculations.appxEqual(pivotAngle, getRotation(), TrunkConstants.ANGLE_DEADZONE) && MiscCalculations.appxEqual(elevatorPosition, getPosition(), TrunkConstants.ELEVATOR_DEADZONE)
    }

    fun getRotation(): Double {
        return frc.robot.util.Math.wrapAroundAngles((-io.getRawRotation() * 360.0) - TrunkConstants.rotationOffset)
    }

    fun getPosition(): Double {
        return io.getRawPosition() * TrunkConstants.ELEVATOR2M
    }

    fun freeMotors() {
        io.setAngleIdleMode(CANSparkBase.IdleMode.kCoast)
        io.setPositionIdleMode(CANSparkBase.IdleMode.kCoast)
        io.setRotationSpeed(0.0)
    }

    fun brakeMotors() {
        io.setPositionIdleMode(CANSparkBase.IdleMode.kBrake)
        io.setAngleIdleMode(CANSparkBase.IdleMode.kBrake)
    }

}
