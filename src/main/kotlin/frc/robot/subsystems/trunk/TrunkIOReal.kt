package frc.robot.subsystems.trunk

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DutyCycleEncoder
import frc.robot.constants.TrunkConstants

class TrunkIOReal : TrunkIO {

    private val elevatorMotor = CANSparkMax(20, CANSparkLowLevel.MotorType.kBrushless)
    private val positionEncoder = elevatorMotor.getAlternateEncoder(8192)

    private val mainRotationMotor = CANSparkMax(22, CANSparkLowLevel.MotorType.kBrushless)
    private val followerRotationMotor = CANSparkMax(21, CANSparkLowLevel.MotorType.kBrushless)

    private val rotationEncoder = DutyCycleEncoder(TrunkConstants.rotationEncoderID)
    private var angleIdleMode = CANSparkBase.IdleMode.kBrake

    private val topLimit = DigitalInput(0)
    private val bottomLimit = DigitalInput(1)

    private var bottomPositionLimit = TrunkConstants.BOTTOM_BREAK_BEAM_POSITION
    private var topPositionLimit = TrunkConstants.TOP_BREAK_BEAM_POSITION
    private var bottomRotationLimit = TrunkConstants.SAFE_TRAVEL_ANGLE
    private var topRotationLimit = TrunkConstants.MAX_ANGLE

    init {
        elevatorMotor.restoreFactoryDefaults()
        mainRotationMotor.restoreFactoryDefaults()
        followerRotationMotor.restoreFactoryDefaults()

        elevatorMotor.inverted = false // elevator likes to not be inverted idk why
        mainRotationMotor.inverted = false

        elevatorMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, false)
        elevatorMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, false)

        followerRotationMotor.follow(mainRotationMotor, true)
    }

    override fun setPositionIdleMode(mode: CANSparkBase.IdleMode) {
        elevatorMotor.setIdleMode(mode)
    }

    override fun setAngleIdleMode(mode: CANSparkBase.IdleMode) {
        mainRotationMotor.setIdleMode(mode)
        followerRotationMotor.setIdleMode(mode)
        angleIdleMode = mode
    }

    override fun getAngleIdleMode(): CANSparkBase.IdleMode = angleIdleMode

    override fun setZeroPosition(top: Boolean) {
        positionEncoder.setPosition(
            if (top) {
                TrunkConstants.TOP_BREAK_BEAM_POSITION * TrunkConstants.M2ELEVATOR
            } else {
                TrunkConstants.BOTTOM_BREAK_BEAM_POSITION * TrunkConstants.M2ELEVATOR
            }
        )
        if (!top)
            elevatorMotor.inverted = !elevatorMotor.inverted
        setTopPositionLimit(TrunkConstants.TOP_BREAK_BEAM_POSITION)
        setBottomPositionLimit(TrunkConstants.BOTTOM_BREAK_BEAM_POSITION)
    }

    override fun atTopLimit(): Boolean = topLimit.get()

    override fun atBottomLimit(): Boolean = bottomLimit.get()

    override fun setPositionLimits(on: Boolean) {
//        elevatorMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, on)
//        elevatorMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, on)
    }

    override fun getRawPosition(): Double = positionEncoder.position

    override fun getRawRotation(): Double = rotationEncoder.absolutePosition

    override fun setTopPositionLimit(position: Double) {
        if (topPositionLimit != position) {
            topPositionLimit = position
            elevatorMotor.setSoftLimit(
                CANSparkBase.SoftLimitDirection.kForward,
                (position * TrunkConstants.M2ELEVATOR).toFloat()
            )
            elevatorMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, true)
        }
    }

    override fun setBottomPositionLimit(position: Double) {
        if (bottomPositionLimit != position) {
            bottomPositionLimit = position
            elevatorMotor.setSoftLimit(
                CANSparkBase.SoftLimitDirection.kReverse,
                (position * TrunkConstants.M2ELEVATOR).toFloat()
            )
            elevatorMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, true)
        }
    }

    override fun setTopRotationLimit(angle: Double) {
        if (topRotationLimit != angle) {
            topRotationLimit = angle
            mainRotationMotor.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward, angle.toFloat())
            mainRotationMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, true)
        }
    }

    override fun setBottomRotationLimit(angle: Double) {
        if (bottomRotationLimit != angle) {
            bottomRotationLimit = angle
            mainRotationMotor.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, angle.toFloat())
            mainRotationMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, true)
        }
    }

    override fun setElevatorSpeed(speed: Double) {
        elevatorMotor.set(-speed)
    }

    override fun setRotationSpeed(speed: Double) {
        mainRotationMotor.set(speed)
    }

    override fun setRotationVoltage(volts: Double) {
        mainRotationMotor.setVoltage(volts);
    }

    override fun periodic() {
    }
}