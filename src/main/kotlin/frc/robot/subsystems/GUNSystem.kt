package frc.robot.subsystems

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.SparkLimitSwitch.Type.kNormallyOpen
import com.revrobotics.CANSparkMax
import com.revrobotics.SparkAbsoluteEncoder
import com.revrobotics.SparkPIDController
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin
import com.revrobotics.CANSparkBase.ControlType.kSmartMotion

enum class GUNPosition {
    AMP,
    SPEAKER,
    INTAKE,
    STOW,
}

class GUNSystem() : SubsystemBase() {
    private val elevatorMotor = CANSparkMax(11, CANSparkLowLevel.MotorType.kBrushless)
    private val leftRotationMotor = CANSparkMax(13, CANSparkLowLevel.MotorType.kBrushless)
    private val rightRotationMotor = CANSparkMax(14, CANSparkLowLevel.MotorType.kBrushless)
    private val positionEncoder = elevatorMotor.getAlternateEncoder(POSITION_GEAR_RATIO)

    private val mainRotationMotor = rightRotationMotor // remember to make other one follow this
    private val followerRotationMotor = leftRotationMotor

    private val rotationEncoder = mainRotationMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle)

    private val leftShooter = CANSparkMax(TODO(), CANSparkLowLevel.MotorType.kBrushless)
    private val rightShooter = CANSparkMax(TODO(), CANSparkLowLevel.MotorType.kBrushless)

    private val positionPID = elevatorMotor.pidController
    private val rotationPID = mainRotationMotor.pidController

    var targetPosition = GUNPosition.SPEAKER
    var rotationSetPoint = TARGET_SAFE_ANGLE
    var positionSetPoint = SPEAKER_POSITION
    var rotationOffset: Double
    var currentDistance: Double
    var shootingAngle = TARGET_SAFE_ANGLE
    var isDefinitelyAboveCrossbar = false

    init {
        leftRotationMotor.restoreFactoryDefaults()
        rightRotationMotor.restoreFactoryDefaults()

        elevatorMotor.inverted = false
        mainRotationMotor.inverted = false

        mainRotationMotor.getForwardLimitSwitch(kNormallyOpen).enableLimitSwitch(false)
        mainRotationMotor.getReverseLimitSwitch(kNormallyOpen).enableLimitSwitch(false)
        elevatorMotor.getForwardLimitSwitch(kNormallyOpen).enableLimitSwitch(false);
        elevatorMotor.getReverseLimitSwitch(kNormallyOpen).enableLimitSwitch(false);

        followerRotationMotor.follow(mainRotationMotor, true)

        mainRotationMotor.setIdleMode(CANSparkBase.IdleMode.kBrake)
        followerRotationMotor.setIdleMode(CANSparkBase.IdleMode.kBrake)
        elevatorMotor.setIdleMode(CANSparkBase.IdleMode.kBrake)

        positionPID = elevatorMotor.pidController
        rotationPID = mainRotationMotor.pidController

        positionPID.setP(positionKP)
        positionPID.setI(positionKI)
        positionPID.setD(positionKD)
        positionPID.setIZone(positionIz)
        positionPID.setFF(positionFF)
        positionPID.setOutputRange(positionMin, positionMax)
        positionPID.setSmartMotionMaxVelocity(positionMaxRPM, SMART_MOTION_SLOT)
        positionPID.setSmartMotionMinOutputVelocity(positionMinRPM, SMART_MOTION_SLOT)
        positionPID.setSmartMotionMaxAccel(positionMaxAcceleration, SMART_MOTION_SLOT)
        positionPID.setSmartMotionAllowedClosedLoopError(positionMaxError, SMART_MOTION_SLOT)

        rotationPID.setP(rotationKP)
        rotationPID.setI(rotationKI)
        rotationPID.setD(rotationKD)
        rotationPID.setIZone(rotationIz)
        rotationPID.setFF(rotationFF)
        rotationPID.setOutputRange(rotationMin, rotationMax)
        rotationPID.setSmartMotionMaxVelocity(rotationMaxRPM, SMART_MOTION_SLOT)
        rotationPID.setSmartMotionMinOutputVelocity(rotationMinRPM, SMART_MOTION_SLOT)
        rotationPID.setSmartMotionMaxAccel(rotationMaxAcceleration, SMART_MOTION_SLOT)
        rotationPID.setSmartMotionAllowedClosedLoopError(rotationMaxError, SMART_MOTION_SLOT)
    }

    fun setZeroPosition() {

    }

    fun goToIntake() {
        targetPosition = GUNPosition.INTAKE
    }

    fun goToAmp() {
        targetPosition = GUNPosition.AMP
    }

    fun setDesiredPosition(position: Double) {
        positionSetPoint = position
        positionPID.setReference(position, kSmartMotion)
    }

    fun setDesiredRotation(angle: Double) {
        rotationSetPoint = angle
        rotationPID.setReference(angle, kSmartMotion)
    }

    fun goToShoot(angle: Double) {
        shootingAngle = angle
        targetPosition = GUNPosition.SPEAKER
    }

    fun setSpeed(left: Double, right: Double) {
        leftShooter.set(left)
        rightShooter.set(right)
    }

    fun getRotation(): Double {
        return rotationEncoder.position
    }

    fun getPosition(): Double {
        return positionEncoder.position
    }

    fun shoot(angle: Double, leftPower: Double, rightPower: Double) {
        goToShoot(angle)
        setSpeed(leftPower, rightPower)
    }


    fun elevate(speed : Double) {
        elevatorMotor.set(speed)
    }

    override fun periodic() {
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
            GUNPosition.AMP -> {
                if(rotationSetPoint != AMP_ANGLE)
                    setDesiredRotation(AMP_ANGLE)
                if(positionSetPoint != AMP_POSITION) {
                    if(isDefinitelyAboveCrossbar || getPosition() > CROSSBAR_TOP) {
                        isDefinitelyAboveCrossbar = true
                        setDesiredPosition(AMP_POSITION)
                    }
                    else if (getRotation() > MIN_SAFE_ANGLE)
                        setDesiredPosition(AMP_POSITION)
                }
            }
            GUNPosition.INTAKE -> {
                isDefinitelyAboveCrossbar = false
                if(rotationSetPoint != INTAKE_ANGLE) {
                    if(getPosition() < CROSSBAR_BOTTOM)
                        setDesiredRotation(INTAKE_ANGLE)
                    else if(rotationSetPoint != TARGET_SAFE_ANGLE)
                        setDesiredRotation(TARGET_SAFE_ANGLE)
                }
                if(positionSetPoint != INTAKE_POSITION) {
                    if(getRotation() > MIN_SAFE_ANGLE)
                        setDesiredPosition(INTAKE_POSITION)
                    else if(positionSetPoint != CROSSBAR_TOP)
                        setDesiredPosition(CROSSBAR_TOP)
                }
            }
            GUNPosition.SPEAKER -> {
                if(rotationSetPoint != shootingAngle) {
                    if(isDefinitelyAboveCrossbar || getPosition() > CROSSBAR_TOP) {
                        isDefinitelyAboveCrossbar = true
                        setDesiredRotation(shootingAngle) // maybe no
                    } else if (rotationSetPoint != TARGET_SAFE_ANGLE)
                        setDesiredRotation(TARGET_SAFE_ANGLE)
                }
                if(positionSetPoint != SPEAKER_POSITION) {
                    if(isDefinitelyAboveCrossbar || getRotation() > MIN_SAFE_ANGLE)
                        setDesiredPosition(SPEAKER_POSITION)
                    else
                        setDesiredPosition(CROSSBAR_BOTTOM)
                }
            }
            GUNPosition.STOW -> {
                if(rotationSetPoint != STOW_ANGLE) {
                    if(isDefinitelyAboveCrossbar || getPosition() > CROSSBAR_TOP) {
                        isDefinitelyAboveCrossbar = true
                        setDesiredRotation(STOW_ANGLE) // maybe no
                    } else if (rotationSetPoint != TARGET_SAFE_ANGLE)
                        setDesiredRotation(TARGET_SAFE_ANGLE)
                }
                if(positionSetPoint != STOW_POSITION) {
                    if(isDefinitelyAboveCrossbar || getRotation() > MIN_SAFE_ANGLE)
                        setDesiredPosition(STOW_POSITION)
                    else
                        setDesiredPosition(CROSSBAR_BOTTOM)
                }
            }
        }
    }

    override fun simulationPeriodic() {
    }

    companion object {
        // someone rename these
        const val POSITION_GEAR_RATIO = 15
        const val ROTATION_GEAR_RATIO = 100
        const val MAX_PIVOT_HEIGHT_M = 0.64446
        const val MIN_PIVOT_HEIGHT_M = 0.348701
        const val THING_LENGTH_M = 0.6133
        const val MOVER_GEAR_RADIUS_M = 0.0127
        val gearCircumfrence = 2 * PI * MOVER_GEAR_RADIUS_M
        private const val ELEVATOR_ANGLE = 28.8309683
        val d2y = sin(ELEVATOR_ANGLE * PI / 180.0)
        val d2x = cos(ELEVATOR_ANGLE * PI / 180.0)
        var MIN_SAFE_ANGLE: Double = TODO()
        var TARGET_SAFE_ANGLE: Double = TODO()
        var MIN_SAFE_DISTANCE: Double = TODO()
        var ABS_MIN_ANGLE: Double = TODO()
        var ABS_MAX_ANGLE: Double = TODO()
        var TOP_M: Double = TODO()
        var BOTTOM_M: Double = TODO()
        var SPEAKER_POSITION: Double = TODO()
        var AMP_POSITION: Double = TODO()
        var AMP_ANGLE: Double = TODO()
        var INTAKE_POSITION: Double = TODO()
        var INTAKE_ANGLE: Double = TODO()
        var CROSSBAR_BOTTOM: Double = TODO()
        var CROSSBAR_TOP: Double = TODO()
        var STOW_POSITION: Double = TODO()
        var STOW_ANGLE: Double = TODO()

        private const val SMART_MOTION_SLOT = 0

        // TUNE!!!!!
        private var positionKP = 5e-5
        private var positionKI = 1e-6
        private var positionKD = 0.0
        private var positionIz = 0.0
        private var positionFF = 0.000156
        private var positionMax = 1.0
        private var positionMin = -1.0
        private var positionMinRPM = 10.0
        private var positionMaxRPM = 5700.0
        private var positionMaxAcceleration = 1500.0
        private var positionMaxError = 5.0

        // TUNE!!!!!
        private var rotationKP = 5e-5
        private var rotationKI = 1e-6
        private var rotationKD = 0.0
        private var rotationIz = 0.0
        private var rotationFF = 0.000156
        private var rotationMin = -1.0
        private var rotationMax = 1.0
        private var rotationMinRPM = 10.0
        private var rotationMaxRPM = 5700.0
        private var rotationMaxAcceleration = 1500.0
        private var rotationMaxError = 5.0
    }
}