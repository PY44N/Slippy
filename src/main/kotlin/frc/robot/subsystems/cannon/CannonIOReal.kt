package frc.robot.subsystems.cannon

import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.Encoder
import frc.robot.RobotContainer
import frc.robot.ShooterState
import frc.robot.constants.CannonConstants

class CannonIOReal : CannonIO {
    private val leftShooterMotor: CANSparkMax =
        CANSparkMax(CannonConstants.LEFT_SHOOTER_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless)
    private val rightShooterMotor: CANSparkMax =
        CANSparkMax(CannonConstants.RIGHT_SHOOTER_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless)

    private val rightShooterEncoder = Encoder(8, 7)
    private val leftShooterEncoder = Encoder(6, 5)

    private val leftShooterPID =
        PIDController(CannonConstants.shooterKP, CannonConstants.shooterKI, CannonConstants.shooterKD)
    private val rightShooterPID =
        PIDController(CannonConstants.shooterKP, CannonConstants.shooterKI, CannonConstants.shooterKD)


    private val outerIntakeMotor: CANSparkMax =
        CANSparkMax(CannonConstants.OUTER_INTAKE_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless)
    private val innerIntakeMotor: CANSparkMax =
        CANSparkMax(CannonConstants.INNER_INTAKE_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless)

    private val entryBeamBreak = DigitalInput(3)
    private val loadedBeamBreak = DigitalInput(2)

    private var desiredRightShooterVel = 0.0
    private var desiredLeftShooterVel = 0.0

    init {
        rightShooterMotor.inverted = true

        rightShooterEncoder.distancePerPulse = 0.017
        leftShooterEncoder.distancePerPulse = 0.017
    }

    override fun getEntryBeamBreak() = !entryBeamBreak.get()

    override fun getLoadedBeamBreak() = !loadedBeamBreak.get()

    override fun setShooterVel(leftRPM: Double, rightRPM: Double) {
        desiredLeftShooterVel = leftRPM
        desiredRightShooterVel = rightRPM
    }

    override fun setInnerIntakePercent(percent: Double) {
        innerIntakeMotor.set(percent)
    }

    override fun setOuterIntakePercent(percent: Double) {
        outerIntakeMotor.set(percent)
    }

    override fun periodic() {
        if (RobotContainer.stateMachine.shooterState != ShooterState.Stopped) {
            val rightShooterFF = desiredRightShooterVel * CannonConstants.shooterFFMultiplier
            val leftShooterFF = desiredLeftShooterVel * CannonConstants.shooterFFMultiplier

            val leftPIDOut = leftShooterPID.calculate(-leftShooterEncoder.rate, desiredLeftShooterVel)
            val rightPIDOut = rightShooterPID.calculate(rightShooterEncoder.rate, desiredRightShooterVel)

            val leftPercent = (leftShooterFF + leftPIDOut) / CannonConstants.SHOOTER_MAX_RPM
            val rightPercent = (rightShooterFF + rightPIDOut) / CannonConstants.SHOOTER_MAX_RPM

            leftShooterMotor.set(leftPercent)
            rightShooterMotor.set(rightPercent)
        }

    }

}