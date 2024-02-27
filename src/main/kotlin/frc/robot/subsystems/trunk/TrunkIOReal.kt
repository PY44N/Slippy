package frc.robot.subsystems.trunk

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.constants.TrunkConstants

class TrunkIOReal : TrunkIO {
    private val elevatorEncoderConversionFactor = 1.0 / TrunkConstants.MOVER_GEAR_CIRCUMFERENCE_M

    private val elevatorMotor = CANSparkMax(15, CANSparkLowLevel.MotorType.kBrushless)
    private val positionEncoder = elevatorMotor.getAlternateEncoder(8192)

    private val mainRotationMotor = CANSparkMax(13, CANSparkLowLevel.MotorType.kBrushless)
    private val followerRotationMotor = CANSparkMax(14, CANSparkLowLevel.MotorType.kBrushless)

    // private val rotationEncoder = DutyCycleEncoder(0)
    private val rotationEncoder = mainRotationMotor.getAlternateEncoder(16384)

    private val topLimit = DigitalInput(1)
    private val bottomLimit = DigitalInput(2)

    private val positionPID = elevatorMotor.pidController
    private val rotationPID = mainRotationMotor.pidController

    init {
        SmartDashboard.putNumber("Rotation P Gain", TrunkConstants.rotationKP)
        SmartDashboard.putNumber("Rotation I Gain", TrunkConstants.rotationKI)
        SmartDashboard.putNumber("Rotation D Gain", TrunkConstants.rotationKD)
        SmartDashboard.putNumber("Rotation I Zone", TrunkConstants.rotationIz)
        SmartDashboard.putNumber("Rotation Feed Forward", TrunkConstants.rotationFF)
        SmartDashboard.putNumber("Rotation Max Output", TrunkConstants.rotationMax)
        SmartDashboard.putNumber("Rotation Min Output", TrunkConstants.rotationMin)
        SmartDashboard.putNumber("Rotation Max Velocity", TrunkConstants.rotationMaxRPM)
        SmartDashboard.putNumber("Rotation Min Velocity", TrunkConstants.rotationMinRPM)
        SmartDashboard.putNumber("Rotation Max Acceleration", TrunkConstants.rotationMaxAcceleration)
        SmartDashboard.putNumber("Rotation Allowed Closed Loop Error", TrunkConstants.rotationMaxError)
        SmartDashboard.putNumber("Position SetPoint", 0.5)
        SmartDashboard.putNumber("Rotation SetPoint", 0.0)

        elevatorMotor.restoreFactoryDefaults()
        mainRotationMotor.restoreFactoryDefaults()
        followerRotationMotor.restoreFactoryDefaults()

        elevatorMotor.inverted = false // elevator likes to not be inverted idk why
        mainRotationMotor.inverted = true

        elevatorMotor.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward, 0.0f)
        elevatorMotor.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse,
                (-.73 * elevatorEncoderConversionFactor).toFloat()
        )
        elevatorMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, false)
        elevatorMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, false)

        rotationEncoder.positionConversionFactor = 360.0

        positionPID.setFeedbackDevice(positionEncoder)
        rotationPID.setFeedbackDevice(rotationEncoder)

        //        mainRotationMotor.getForwardLimitSwitch(kNormallyOpen).enableLimitSwitch(false)
        //        mainRotationMotor.getReverseLimitSwitch(kNormallyOpen).enableLimitSwitch(false)
        //        elevatorMotor.getForwardLimitSwitch(kNormallyOpen).enableLimitSwitch(false)
        //        elevatorMotor.getReverseLimitSwitch(kNormallyOpen).enableLimitSwitch(false)

        followerRotationMotor.follow(mainRotationMotor, true)

        mainRotationMotor.setIdleMode(CANSparkBase.IdleMode.kBrake)
        followerRotationMotor.setIdleMode(CANSparkBase.IdleMode.kBrake)
        elevatorMotor.setIdleMode(CANSparkBase.IdleMode.kBrake)

        positionPID.setP(TrunkConstants.positionKP)
        positionPID.setI(TrunkConstants.positionKI)
        positionPID.setD(TrunkConstants.positionKD)
        positionPID.setIZone(TrunkConstants.positionIz)
        positionPID.setFF(TrunkConstants.positionFF)
        positionPID.setOutputRange(TrunkConstants.positionMin, TrunkConstants.positionMax)
        positionPID.setSmartMotionMaxVelocity(TrunkConstants.positionMaxRPM, TrunkConstants.SMART_MOTION_SLOT)
        positionPID.setSmartMotionMinOutputVelocity(TrunkConstants.positionMinRPM, TrunkConstants.SMART_MOTION_SLOT)
        positionPID.setSmartMotionMaxAccel(TrunkConstants.positionMaxAcceleration, TrunkConstants.SMART_MOTION_SLOT)
        positionPID.setSmartMotionAllowedClosedLoopError(TrunkConstants.positionMaxError, TrunkConstants.SMART_MOTION_SLOT)

        rotationPID.setP(TrunkConstants.rotationKP)
        rotationPID.setI(TrunkConstants.rotationKI)
        rotationPID.setD(TrunkConstants.rotationKD)
        rotationPID.setIZone(TrunkConstants.rotationIz)
        rotationPID.setFF(TrunkConstants.rotationFF)
        rotationPID.setOutputRange(TrunkConstants.rotationMin, TrunkConstants.rotationMax)
        rotationPID.setSmartMotionMaxVelocity(TrunkConstants.rotationMaxRPM, TrunkConstants.SMART_MOTION_SLOT)
        rotationPID.setSmartMotionMinOutputVelocity(TrunkConstants.rotationMinRPM, TrunkConstants.SMART_MOTION_SLOT)
        rotationPID.setSmartMotionMaxAccel(TrunkConstants.rotationMaxAcceleration, TrunkConstants.SMART_MOTION_SLOT)
        rotationPID.setSmartMotionAllowedClosedLoopError(TrunkConstants.rotationMaxError, TrunkConstants.SMART_MOTION_SLOT)
    }

    override fun setZeroPosition() {
        positionEncoder.setPosition(-TrunkConstants.TOP_LIMIT_SWITCH_POSITION * elevatorEncoderConversionFactor)
        elevatorMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, true)
        elevatorMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, true)
    }

    override fun atTopLimit(): Boolean {
        return topLimit.get()
    }

    override fun calibrate() {
        elevatorMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, false)
        elevatorMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, false)
    }

    override fun setZeroRotation() {
        rotationEncoder.setPosition(0.0)
    }

    override fun getPosition(): Double {
        return -positionEncoder.position * TrunkConstants.MOVER_GEAR_CIRCUMFERENCE_M
    }

    override fun getRotation(): Double {
        return rotationEncoder.position
    }

    override fun setDesiredPosition(position: Double) {
        positionPID.setReference(-position * elevatorEncoderConversionFactor, CANSparkBase.ControlType.kPosition)
    }

    override fun setDesiredRotation(angle: Double) {
        rotationPID.setReference(angle, CANSparkBase.ControlType.kPosition)
    }


    override fun setElevatorSpeed(speed: Double) {
        elevatorMotor.set(-speed)
    }

    override fun setRotationSpeed(speed: Double) {
        mainRotationMotor.set(speed)
    }

    override fun periodic() {
        SmartDashboard.putNumber("elevator position", getPosition())
        SmartDashboard.putNumber("pivot rotation", getRotation())
        SmartDashboard.putNumber("follower voltage", followerRotationMotor.appliedOutput)
        SmartDashboard.putNumber("leader voltage", mainRotationMotor.appliedOutput)
    }

}