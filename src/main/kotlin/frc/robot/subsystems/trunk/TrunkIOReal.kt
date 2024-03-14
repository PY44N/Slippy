package frc.robot.subsystems.trunk

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.CoastOut
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DutyCycleEncoder
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.constants.TrunkConstants

class TrunkIOReal : TrunkIO {

    private val elevatorMotor = CANSparkMax(20, CANSparkLowLevel.MotorType.kBrushless)
    private val positionEncoder = elevatorMotor.getAlternateEncoder(8192)

    private val mainRotationMotor = TalonFX(22) // Right Motor
    private val followerRotationMotor = TalonFX(21) // Left Motor

    private val rotationEncoder = DutyCycleEncoder(TrunkConstants.rotationEncoderID)

    private val topLimit = DigitalInput(0)

    override var positionBrake = true
        set(enabled) {
            if (positionBrake != enabled)
                elevatorMotor.setIdleMode(if (enabled) CANSparkBase.IdleMode.kBrake else CANSparkBase.IdleMode.kCoast)
            field = enabled
        }
    override var rotationBrake = true
        set(enabled) {
            if (field != enabled) {
                mainRotationMotor.setNeutralMode(if (enabled) NeutralModeValue.Brake else NeutralModeValue.Coast)
                followerRotationMotor.setNeutralMode(if (enabled) NeutralModeValue.Brake else NeutralModeValue.Coast)
            }
            field = enabled
        }

    init {
        // factory reset to make it not be bad
        elevatorMotor.restoreFactoryDefaults()
        mainRotationMotor.configurator.apply(TalonFXConfiguration())
        followerRotationMotor.configurator.apply(TalonFXConfiguration())

        elevatorMotor.inverted = false // elevator likes to not be inverted idk why
        mainRotationMotor.inverted = TODO()

        // ensure motors are initially braked
        elevatorMotor.setIdleMode(CANSparkBase.IdleMode.kBrake)
        mainRotationMotor.setNeutralMode(NeutralModeValue.Brake)
        followerRotationMotor.setNeutralMode(NeutralModeValue.Brake)

        followerRotationMotor.setControl(Follower(22, true))
    }

    override fun setZeroPosition() {
        positionEncoder.setPosition(TrunkConstants.TOP_BREAK_BEAM_POSITION * TrunkConstants.M2ELEVATOR)
    }

    override fun atTopLimit(): Boolean = topLimit.get()

    override fun getRawPosition(): Double = positionEncoder.position

    override fun getRawRotation(): Double = rotationEncoder.absolutePosition

    override fun setElevatorSpeed(speed: Double) {
        SmartDashboard.putNumber("set elevator speed: ", speed)
        elevatorMotor.set(-speed)
    }

    override fun setRotationVoltage(volts: Double) {
        SmartDashboard.putNumber("set rotation voltage: ", MathUtil.clamp(volts, TrunkConstants.MIN_ROT_VOLTS, TrunkConstants.MAX_ROT_VOLTS))
        mainRotationMotor.setVoltage(MathUtil.clamp(volts, TrunkConstants.MIN_ROT_VOLTS, TrunkConstants.MAX_ROT_VOLTS));
    }

    override fun periodic() {
    }
}