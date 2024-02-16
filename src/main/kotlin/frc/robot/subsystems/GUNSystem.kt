package frc.robot.subsystems

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import com.revrobotics.SparkAbsoluteEncoder
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DutyCycleEncoder
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.constants.GUNConstants
enum class GUNPosition {
    AMP,
    SPEAKER,
    INTAKE,
    STOW,
    TRAP,
    CALIBRATE,
}

class GUNSystem : SubsystemBase() {
    private val elevatorMotor = CANSparkMax(15, CANSparkLowLevel.MotorType.kBrushless)
    private val positionEncoder = elevatorMotor.getAlternateEncoder(GUNConstants.POSITION_GEAR_RATIO)

    private val mainRotationMotor = CANSparkMax(13, CANSparkLowLevel.MotorType.kBrushless)
    private val followerRotationMotor = CANSparkMax(14, CANSparkLowLevel.MotorType.kBrushless)

    private val rotationEncoder = DutyCycleEncoder(0)

    private val topLimit = DigitalInput(1)

//    private val leftShooter = CANSparkMax(TODO(), CANSparkLowLevel.MotorType.kBrushless)
//    private val rightShooter = CANSparkMax(TODO(), CANSparkLowLevel.MotorType.kBrushless)

    private val positionPID = elevatorMotor.pidController
    private val rotationPID = mainRotationMotor.pidController

    var targetPosition = GUNPosition.SPEAKER

    var rotationSetPoint = GUNConstants.TARGET_SAFE_ANGLE
    var positionSetPoint = GUNConstants.SPEAKER_POSITION
    var shootingAngle = GUNConstants.TARGET_SAFE_ANGLE

    var isDefinitelyAboveCrossbar = false

    var calibrationMoving = false

    init {
        elevatorMotor.restoreFactoryDefaults()
        mainRotationMotor.restoreFactoryDefaults()
        followerRotationMotor.restoreFactoryDefaults()

        elevatorMotor.inverted = true
        mainRotationMotor.inverted = false

        positionPID.setFeedbackDevice(positionEncoder)

//        mainRotationMotor.getForwardLimitSwitch(kNormallyOpen).enableLimitSwitch(false)
//        mainRotationMotor.getReverseLimitSwitch(kNormallyOpen).enableLimitSwitch(false)
//        elevatorMotor.getForwardLimitSwitch(kNormallyOpen).enableLimitSwitch(false)
//        elevatorMotor.getReverseLimitSwitch(kNormallyOpen).enableLimitSwitch(false)

        followerRotationMotor.follow(mainRotationMotor, true)

        mainRotationMotor.setIdleMode(CANSparkBase.IdleMode.kBrake)
        followerRotationMotor.setIdleMode(CANSparkBase.IdleMode.kBrake)
        elevatorMotor.setIdleMode(CANSparkBase.IdleMode.kBrake)

//        positionPID.setP(GUNConstants.positionKP)
//        positionPID.setI(GUNConstants.positionKI)
//        positionPID.setD(GUNConstants.positionKD)
//        positionPID.setIZone(GUNConstants.positionIz)
//        positionPID.setFF(GUNConstants.positionFF)
//        positionPID.setOutputRange(GUNConstants.positionMin, GUNConstants.positionMax)
//        positionPID.setSmartMotionMaxVelocity(GUNConstants.positionMaxRPM, GUNConstants.SMART_MOTION_SLOT)
//        positionPID.setSmartMotionMinOutputVelocity(GUNConstants.positionMinRPM, GUNConstants.SMART_MOTION_SLOT)
//        positionPID.setSmartMotionMaxAccel(GUNConstants.positionMaxAcceleration, GUNConstants.SMART_MOTION_SLOT)
//        positionPID.setSmartMotionAllowedClosedLoopError(GUNConstants.positionMaxError, GUNConstants.SMART_MOTION_SLOT)
//
//        rotationPID.setP(GUNConstants.rotationKP)
//        rotationPID.setI(GUNConstants.rotationKI)
//        rotationPID.setD(GUNConstants.rotationKD)
//        rotationPID.setIZone(GUNConstants.rotationIz)
//        rotationPID.setFF(GUNConstants.rotationFF)
//        rotationPID.setOutputRange(GUNConstants.rotationMin, GUNConstants.rotationMax)
//        rotationPID.setSmartMotionMaxVelocity(GUNConstants.rotationMaxRPM, GUNConstants.SMART_MOTION_SLOT)
//        rotationPID.setSmartMotionMinOutputVelocity(GUNConstants.rotationMinRPM, GUNConstants.SMART_MOTION_SLOT)
//        rotationPID.setSmartMotionMaxAccel(GUNConstants.rotationMaxAcceleration, GUNConstants.SMART_MOTION_SLOT)
//        rotationPID.setSmartMotionAllowedClosedLoopError(GUNConstants.rotationMaxError, GUNConstants.SMART_MOTION_SLOT)
    }

    fun setZeroPosition() {
        positionEncoder.setPosition(0.0)
    }

    fun setDesiredPosition(position: Double) {
        positionSetPoint = position
        positionPID.setReference(position, CANSparkBase.ControlType.kSmartMotion)
    }

    fun setDesiredRotation(angle: Double) {
        rotationSetPoint = angle
        rotationPID.setReference(angle + GUNConstants.rotationOffset, CANSparkBase.ControlType.kSmartMotion)
    }
//
//    fun goToShoot(angle: Double) {
//        shootingAngle = angle
//        targetPosition = GUNPosition.SPEAKER
//    }

//    fun setSpeed(left: Double, right: Double) {
//        leftShooter.set(left)
//        rightShooter.set(right)
//    }

    fun getRotation(): Double {
//        return 69.0
        return rotationEncoder.absolutePosition * 360.0
    }

    fun getPosition(): Double {
        return -positionEncoder.position * GUNConstants.MOVER_GEAR_CIRCUMFERENCE_M
    }

//    fun shoot(angle: Double, leftPower: Double, rightPower: Double) {
//        goToShoot(angle)
//        setSpeed(leftPower, rightPower)
//    }


    fun elevate(speed: Double) {
        elevatorMotor.set(speed)
    }

    fun rotate(speed: Double) {
        mainRotationMotor.set(speed)
    }

    override fun periodic() {
        SmartDashboard.putNumber("elevator position", getPosition())
        SmartDashboard.putNumber("pivot rotation", getRotation())
        /*
        val inputElevatorP = SmartDashboard.getNumber("Elevator P Gain", 0.0);
        val inputElevatorI = SmartDashboard.getNumber("Elevator I Gain", 0.0);
        val inputElevatorD = SmartDashboard.getNumber("Elevator D Gain", 0.0);
        val inputElevatorIz = SmartDashboard.getNumber("Elevator I Zone", 0.0);
        val inputElevatorFf = SmartDashboard.getNumber("Elevator Feed Forward", 0.0);
        val inputElevatorMax = SmartDashboard.getNumber("Elevator Max Output", 0.0);
        val inputElevatorMin = SmartDashboard.getNumber("Elevator Min Output", 0.0);
        val inputElevatorMaxV = SmartDashboard.getNumber("Elevator Max Velocity", 0.0);
        val inputElevatorMinV = SmartDashboard.getNumber("Elevator Min Velocity", 0.0);
        val inputElevatorMaxA = SmartDashboard.getNumber("Elevator Max Acceleration", 0.0);
        val inputElevatorAllE = SmartDashboard.getNumber("Elevator Allowed Closed Loop Error", 0.0);

        if(inputElevatorP != positionKP) {
            positionPID.setP(inputElevatorP)
            positionKP = inputElevatorP
        }
        if(inputElevatorI != positionKI) {
            positionPID.setI(inputElevatorI)
            positionKI = inputElevatorI
        }
        if(inputElevatorD != positionKD) {
            positionPID.setD(inputElevatorD)
            positionKD = inputElevatorD
        }
        if(inputElevatorIz != positionIz) {
            positionPID.setIZone(inputElevatorIz)
            positionIz = inputElevatorIz
        }
        if(inputElevatorFf != positionFF) {
            positionPID.setFF(inputElevatorFf)
            positionFF = inputElevatorFf
        }
        if((inputElevatorMax != positionMax) || (inputElevatorMin != positionMin)) {
            positionPID.setOutputRange(inputElevatorMin, inputElevatorMax)
            positionMin = inputElevatorMin
            positionMax = inputElevatorMax
        }
        if(inputElevatorMaxV != positionMaxRPM) {
            positionPID.setSmartMotionMaxVelocity(inputElevatorMaxV, SMART_MOTION_SLOT)
            positionMaxRPM = inputElevatorMaxV
        }
        if(inputElevatorMaxA != positionMaxAcceleration) {
            positionPID.setSmartMotionMaxAccel(inputElevatorMaxA, SMART_MOTION_SLOT)
            positionMaxAcceleration = inputElevatorMaxA
        }
        if(inputElevatorMinV != positionMinRPM) {
            positionPID.setSmartMotionMinOutputVelocity(inputElevatorMinV, SMART_MOTION_SLOT)
            positionMinRPM = inputElevatorMinV
        }
        if(inputElevatorAllE != positionMaxError) {
            positionPID.setSmartMotionAllowedClosedLoopError(inputElevatorAllE, SMART_MOTION_SLOT)
            positionMaxError = inputElevatorAllE
        }

        val inputRotationP = SmartDashboard.getNumber("Rotation P Gain", 0.0);
        val inputRotationI = SmartDashboard.getNumber("Rotation I Gain", 0.0);
        val inputRotationD = SmartDashboard.getNumber("Rotation D Gain", 0.0);
        val inputRotationIz = SmartDashboard.getNumber("Rotation I Zone", 0.0);
        val inputRotationFf = SmartDashboard.getNumber("Rotation Feed Forward", 0.0);
        val inputRotationMax = SmartDashboard.getNumber("Rotation Max Output", 0.0);
        val inputRotationMin = SmartDashboard.getNumber("Rotation Min Output", 0.0);
        val inputRotationMaxV = SmartDashboard.getNumber("Rotation Max Velocity", 0.0);
        val inputRotationMinV = SmartDashboard.getNumber("Rotation Min Velocity", 0.0);
        val inputRotationMaxA = SmartDashboard.getNumber("Rotation Max Acceleration", 0.0);
        val inputRotationAllE = SmartDashboard.getNumber("Rotation Allowed Closed Loop Error", 0.0);

        if(inputRotationP != rotationKP) {
            rotationPID.setP(inputRotationP)
            rotationKP = inputRotationP
        }
        if(inputRotationI != rotationKI) {
            rotationPID.setI(inputRotationI)
            rotationKI = inputRotationI
        }
        if(inputRotationD != rotationKD) {
            rotationPID.setD(inputRotationD)
            rotationKD = inputRotationD
        }
        if(inputRotationIz != rotationIz) {
            rotationPID.setIZone(inputRotationIz)
            rotationIz = inputRotationIz
        }
        if(inputRotationFf != rotationFF) {
            rotationPID.setFF(inputRotationFf)
            rotationFF = inputRotationFf
        }
        if((inputRotationMax != rotationMax) || (inputRotationMin != positionMin)) {
            rotationPID.setOutputRange(inputRotationMin, inputRotationMax)
            rotationMin = inputRotationMin
            rotationMax = inputRotationMax
        }
        if(inputRotationMaxV != rotationMaxRPM) {
            rotationPID.setSmartMotionMaxVelocity(inputRotationMaxV, SMART_MOTION_SLOT)
            rotationMaxRPM = inputRotationMaxV
        }
        if(inputRotationMaxA != rotationMaxAcceleration) {
            rotationPID.setSmartMotionMaxAccel(inputRotationMaxA, SMART_MOTION_SLOT)
            rotationMaxAcceleration = inputRotationMaxA
        }
        if(inputRotationMinV != rotationMinRPM) {
            rotationPID.setSmartMotionMinOutputVelocity(inputRotationMinV, SMART_MOTION_SLOT)
            rotationMinRPM = inputRotationMinV
        }
        if(inputRotationAllE != rotationMaxError) {
            rotationPID.setSmartMotionAllowedClosedLoopError(inputRotationAllE, SMART_MOTION_SLOT)
            rotationMaxError = inputRotationAllE
        }

        val rotationSetPoint = SmartDashboard.getNumber("Rotation Set Position", 0.0)
        rotationPID.setReference(rotationSetPoint, CANSparkBase.ControlType.kSmartMotion)
        val rotation = rotationEncoder.position

        val setPoint = SmartDashboard.getNumber("Set Position", 0.0)
        positionPID.setReference(setPoint, CANSparkBase.ControlType.kSmartMotion)
        val pos = positionEncoder.position

        SmartDashboard.putNumber("Elevator SetPoint", setPoint)
        SmartDashboard.putNumber("Elevator Position", pos)
        SmartDashboard.putNumber("Elevator Motor Output", elevatorMotor.getAppliedOutput())
        SmartDashboard.putNumber("Rotation SetPoint", setPoint)
        SmartDashboard.putNumber("Rotation Position", pos)
        SmartDashboard.putNumber("Rotation Motor Output", mainRotationMotor.getAppliedOutput())
        */
        when(targetPosition) {
            GUNPosition.TRAP -> { goToTrap() }
            GUNPosition.AMP -> { goToAmp()
                if(rotationSetPoint != GUNConstants.AMP_ANGLE)
                    setDesiredRotation(GUNConstants.AMP_ANGLE)
                if(positionSetPoint != GUNConstants.AMP_POSITION) {
                    if(isDefinitelyAboveCrossbar || getPosition() > GUNConstants.CROSSBAR_TOP) {
                        isDefinitelyAboveCrossbar = true
                        setDesiredPosition(GUNConstants.AMP_POSITION)
                    }
                    else if (getRotation() > GUNConstants.MIN_SAFE_ANGLE)
                        setDesiredPosition(GUNConstants.AMP_POSITION)
                }
            }
            GUNPosition.INTAKE -> { goToIntakePose()
                isDefinitelyAboveCrossbar = false
                if(rotationSetPoint != GUNConstants.INTAKE_ANGLE) {
                    if(getPosition() < GUNConstants.CROSSBAR_BOTTOM)
                        setDesiredRotation(GUNConstants.INTAKE_ANGLE)
                    else if(rotationSetPoint != GUNConstants.TARGET_SAFE_ANGLE)
                        setDesiredRotation(GUNConstants.TARGET_SAFE_ANGLE)
                }
                if(positionSetPoint != GUNConstants.INTAKE_POSITION) {
                    if(getRotation() > GUNConstants.MIN_SAFE_ANGLE)
                        setDesiredPosition(GUNConstants.INTAKE_POSITION)
                    else if(positionSetPoint != GUNConstants.CROSSBAR_TOP)
                        setDesiredPosition(GUNConstants.CROSSBAR_TOP)
                }
            }
            GUNPosition.SPEAKER -> {
                if(rotationSetPoint != shootingAngle) {
                    if(isDefinitelyAboveCrossbar || getPosition() > GUNConstants.CROSSBAR_TOP) {
                        isDefinitelyAboveCrossbar = true
                        setDesiredRotation(shootingAngle) // maybe no
                    } else if (rotationSetPoint != GUNConstants.TARGET_SAFE_ANGLE)
                        setDesiredRotation(GUNConstants.TARGET_SAFE_ANGLE)
                }
                if(positionSetPoint != GUNConstants.SPEAKER_POSITION) {
                    if(isDefinitelyAboveCrossbar || getRotation() > GUNConstants.MIN_SAFE_ANGLE)
                        setDesiredPosition(GUNConstants.SPEAKER_POSITION)
                    else
                        setDesiredPosition(GUNConstants.CROSSBAR_BOTTOM)
                }
            }
            GUNPosition.STOW -> {
                if(rotationSetPoint != GUNConstants.STOW_ANGLE) {
                    if(isDefinitelyAboveCrossbar || getPosition() > GUNConstants.CROSSBAR_TOP) {
                        isDefinitelyAboveCrossbar = true
                        setDesiredRotation(GUNConstants.STOW_ANGLE) // maybe no
                    } else if (rotationSetPoint != GUNConstants.TARGET_SAFE_ANGLE)
                        setDesiredRotation(GUNConstants.TARGET_SAFE_ANGLE)
                }
                if(positionSetPoint != GUNConstants.STOW_POSITION) {
                    if(isDefinitelyAboveCrossbar || getRotation() > GUNConstants.MIN_SAFE_ANGLE)
                        setDesiredPosition(GUNConstants.STOW_POSITION)
                    else
                        setDesiredPosition(GUNConstants.CROSSBAR_BOTTOM)
                }
            }
        }
    }
    private fun calibrate() {
        if(rotationSetPoint != GUNConstants.TARGET_SAFE_ANGLE)
            setDesiredRotation(GUNConstants.TARGET_SAFE_ANGLE)
        else if(calibrationMoving == false && getRotation() > GUNConstants.MIN_SAFE_ANGLE)
            elevatorMotor.set(.2)
        if(topLimit.get()) {
            targetPosition = GUNPosition.STOW
            setZeroPosition()
        }
    }
    private fun goToAmpPose() {

    }
    private fun goToSpeakerPose() {

    }
    private fun goToStowPose() {

    }

    private fun goToTrapPose() {

    }

    private fun goToIntakePose() {

    }

    fun ampScore() {

    }

    fun intake() {

    }



    override fun simulationPeriodic() {}
}