package frc.robot.subsystems.trunk

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import com.revrobotics.SparkAbsoluteEncoder
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.constants.TrunkConstants

class TrunkIOReal : TrunkIO {
    private val elevatorMotor = CANSparkMax(15, CANSparkLowLevel.MotorType.kBrushless)
    private val positionEncoder = elevatorMotor.getAlternateEncoder(8192)

    private val mainRotationMotor = CANSparkMax(13, CANSparkLowLevel.MotorType.kBrushless)
    private val followerRotationMotor = CANSparkMax(14, CANSparkLowLevel.MotorType.kBrushless)

    private val rotationEncoder = mainRotationMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle)

    private val topLimit = DigitalInput(1)
    private val bottomLimit = DigitalInput(2)

    private val positionPID = elevatorMotor.pidController
    private val rotationPID = mainRotationMotor.pidController

    private var bottomPositionLimit = TrunkConstants.BOTTOM_BREAK_BEAM_POSITION
    private var topPositionLimit = TrunkConstants.TOP_BREAK_BEAM_POSITION
    private var bottomRotationLimit = TrunkConstants.MIN_SAFE_ANGLE
    private var topRotationLimit = TrunkConstants.MAX_ANGLE

    init {
//        SmartDashboard.putNumber("Rotation P Gain", TrunkConstants.rotationKP)
//        SmartDashboard.putNumber("Rotation I Gain", TrunkConstants.rotationKI)
//        SmartDashboard.putNumber("Rotation D Gain", TrunkConstants.rotationKD)
//        SmartDashboard.putNumber("Rotation I Zone", TrunkConstants.rotationIz)
//        SmartDashboard.putNumber("Rotation Feed Forward", TrunkConstants.rotationFF)
//        SmartDashboard.putNumber("Rotation Max Output", TrunkConstants.rotationMax)
//        SmartDashboard.putNumber("Rotation Min Output", TrunkConstants.rotationMin)
//        SmartDashboard.putNumber("Rotation Max Velocity", TrunkConstants.rotationMaxRPM)
//        SmartDashboard.putNumber("Rotation Min Velocity", TrunkConstants.rotationMinRPM)
//        SmartDashboard.putNumber("Rotation Max Acceleration", TrunkConstants.rotationMaxAcceleration)
//        SmartDashboard.putNumber("Rotation Allowed Closed Loop Error", TrunkConstants.rotationMaxError)
//        SmartDashboard.putNumber("Position SetPoint", 0.5)
//        SmartDashboard.putNumber("Rotation SetPoint", 0.0)

        elevatorMotor.restoreFactoryDefaults()
        mainRotationMotor.restoreFactoryDefaults()
        followerRotationMotor.restoreFactoryDefaults()

        elevatorMotor.inverted = false // elevator likes to not be inverted idk why
        mainRotationMotor.inverted = true

        rotationEncoder.positionConversionFactor = 360.0
        positionEncoder.positionConversionFactor = -1.0 / TrunkConstants.MOVER_GEAR_CIRCUMFERENCE_M

        positionPID.setFeedbackDevice(positionEncoder)
        rotationPID.setFeedbackDevice(rotationEncoder)

        elevatorMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, false)
        elevatorMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, false)

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

    override fun setZeroPosition(top: Boolean) {
        positionEncoder.setPosition(if(top) {TrunkConstants.TOP_BREAK_BEAM_POSITION} else {TrunkConstants.BOTTOM_BREAK_BEAM_POSITION})
        if(!top)
            elevatorMotor.inverted = !elevatorMotor.inverted
        setTopPositionLimit(TrunkConstants.TOP_BREAK_BEAM_POSITION)
        setBottomPositionLimit(TrunkConstants.BOTTOM_BREAK_BEAM_POSITION)
    }

    override fun atTopLimit(): Boolean {
        return topLimit.get()
    }

    override fun atBottomLimit(): Boolean {
        return bottomLimit.get()
    }

    override fun disablePositionLimits() {
        elevatorMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, false)
        elevatorMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, false)
    }

    override fun getPosition(): Double {
        return -positionEncoder.position * TrunkConstants.MOVER_GEAR_CIRCUMFERENCE_M
    }

    override fun getRawRotation(): Double {
        return getRotation()
    }

    override fun getRotation(): Double {
        return rotationEncoder.position
    }

    override fun setDesiredPosition(position: Double) {
        positionPID.setReference(position, CANSparkBase.ControlType.kPosition)
    }

    override fun setDesiredRotation(angle: Double) {
        rotationPID.setReference(angle, CANSparkBase.ControlType.kPosition)
    }
    override fun setTopPositionLimit(position: Double) {
        if(topPositionLimit != position) {
            topPositionLimit = position
            elevatorMotor.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward, -position.toFloat())
            elevatorMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, true)
        }
    }

    override fun setBottomPositionLimit(position: Double) {
        if(bottomPositionLimit != position) {
            bottomPositionLimit = position
            elevatorMotor.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, -position.toFloat())
            elevatorMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, true)
        }
    }

    override fun setTopRotationLimit(angle: Double) {
        if(topRotationLimit != angle) {
            topRotationLimit = angle
            mainRotationMotor.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward, angle.toFloat())
            mainRotationMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, true)
        }
    }

    override fun setBottomRotationLimit(angle: Double) {
        if(bottomRotationLimit != angle) {
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

    override fun periodic() {
        SmartDashboard.putNumber("follower voltage", followerRotationMotor.appliedOutput)
        SmartDashboard.putNumber("leader voltage", mainRotationMotor.appliedOutput)
    }

}