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

enum class GUNPosition {
    AMP,
    SPEAKER,
    INTAKE
}

class GUNSystem() : SubsystemBase() {
    private val elevatorMotor = CANSparkMax(11, CANSparkLowLevel.MotorType.kBrushless)
    private val leftPivotMotor = CANSparkMax(13, CANSparkLowLevel.MotorType.kBrushless)
    private val rightPivotMotor = CANSparkMax(14, CANSparkLowLevel.MotorType.kBrushless)
    private val positionEncoder = elevatorMotor.getAlternateEncoder(POSITION_GEAR_RATIO)

    private val mainPivotMotor = rightPivotMotor // remember to make other one follow this
    private val followerPivotMotor = leftPivotMotor

    private val pivotEncoder = mainPivotMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle)

    private val leftShooter = CANSparkMax(TODO(), CANSparkLowLevel.MotorType.kBrushless)
    private val rightShooter = CANSparkMax(TODO(), CANSparkLowLevel.MotorType.kBrushless)

    private val pivotPID: SparkPIDController
    private val positionPID: SparkPIDController


    var currentPosition: GUNPosition
    var currentDistance: Double
    var currentAngle: Double

    init {
        leftPivotMotor.restoreFactoryDefaults()
        rightPivotMotor.restoreFactoryDefaults()

        elevatorMotor.inverted = false
        mainPivotMotor.inverted = false

        mainPivotMotor.getForwardLimitSwitch(kNormallyOpen).enableLimitSwitch(false)
        mainPivotMotor.getReverseLimitSwitch(kNormallyOpen).enableLimitSwitch(false)

        followerPivotMotor.follow(mainPivotMotor, true)

        positionPID = elevatorMotor.pidController
        pivotPID = mainPivotMotor.pidController

        positionPID.setP(pivotKP)
        positionPID.setI(pivotKI)
        positionPID.setD(pivotKD)
        positionPID.setIZone(pivotIz)
        positionPID.setFF(pivotFF)
        positionPID.setOutputRange(positionMin, positionMax)
        positionPID.setSmartMotionMaxVelocity(pivotMaxRPM, SMART_MOTION_SLOT)
        positionPID.setSmartMotionMinOutputVelocity(pivotMinRPM, SMART_MOTION_SLOT)
        positionPID.setSmartMotionMaxAccel(pivotMaxAcceleration, SMART_MOTION_SLOT)
        positionPID.setSmartMotionAllowedClosedLoopError(pivotMaxError, SMART_MOTION_SLOT)

        pivotPID.setP(pivotKP)
        pivotPID.setI(pivotKI)
        pivotPID.setD(pivotKD)
        pivotPID.setIZone(pivotIz)
        pivotPID.setFF(pivotFF)
        pivotPID.setOutputRange(pivotMin, pivotMax)
        pivotPID.setSmartMotionMaxVelocity(pivotMaxRPM, SMART_MOTION_SLOT)
        pivotPID.setSmartMotionMinOutputVelocity(pivotMinRPM, SMART_MOTION_SLOT)
        pivotPID.setSmartMotionMaxAccel(pivotMaxAcceleration, SMART_MOTION_SLOT)
        pivotPID.setSmartMotionAllowedClosedLoopError(pivotMaxError, SMART_MOTION_SLOT)
    }

    fun setZeroPosition() {

    }

    fun goToAngle(angle: Double) {

    }

    fun goToDistance(distance: Double) {

    }

    fun goToIntake() {

    }

    fun goToAmp() {

    }

    fun goToShoot(angle: Double) {

    }

    fun setSpeed(left: Double, right: Double) {
        leftShooter.set(left)
        rightShooter.set(right)
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

        val setPoint = SmartDashboard.getNumber("Set Position", 0.0)
        positionPID.setReference(setPoint, CANSparkBase.ControlType.kSmartMotion)
        val pos = positionEncoder.position

        SmartDashboard.putNumber("Elevator SetPoint", setPoint)
        SmartDashboard.putNumber("Elevator Position", pos)
        SmartDashboard.putNumber("Elevator Motor Output", elevatorMotor.getAppliedOutput())

        val inputPivotP = SmartDashboard.getNumber("Pivot P Gain", 0.0);
        val inputPivotI = SmartDashboard.getNumber("Pivot I Gain", 0.0);
        val inputPivotD = SmartDashboard.getNumber("Pivot D Gain", 0.0);
        val inputPivotIz = SmartDashboard.getNumber("Pivot I Zone", 0.0);
        val inputPivotFf = SmartDashboard.getNumber("Pivot Feed Forward", 0.0);
        val inputPivotMax = SmartDashboard.getNumber("Pivot Max Output", 0.0);
        val inputPivotMin = SmartDashboard.getNumber("Pivot Min Output", 0.0);
        val inputPivotMaxV = SmartDashboard.getNumber("Pivot Max Velocity", 0.0);
        val inputPivotMinV = SmartDashboard.getNumber("Pivot Min Velocity", 0.0);
        val inputPivotMaxA = SmartDashboard.getNumber("Pivot Max Acceleration", 0.0);
        val inputPivotAllE = SmartDashboard.getNumber("Pivot Allowed Closed Loop Error", 0.0);

        if(inputPivotP != pivotKP) {
            pivotPID.setP(inputPivotP)
            pivotKP = inputPivotP
        }
        if(inputPivotI != pivotKI) {
            pivotPID.setI(inputPivotI)
            pivotKI = inputPivotI
        }
        if(inputPivotD != pivotKD) {
            pivotPID.setD(inputPivotD)
            pivotKD = inputPivotD
        }
        if(inputPivotIz != pivotIz) {
            pivotPID.setIZone(inputPivotIz)
            pivotIz = inputPivotIz
        }
        if(inputPivotFf != pivotFF) {
            pivotPID.setFF(inputPivotFf)
            pivotFF = inputPivotFf
        }
        if((inputPivotMax != pivotMax) || (inputPivotMin != positionMin)) {
            pivotPID.setOutputRange(inputPivotMin, inputPivotMax)
            pivotMin = inputPivotMin
            pivotMax = inputPivotMax
        }
        if(inputPivotMaxV != pivotMaxRPM) {
            pivotPID.setSmartMotionMaxVelocity(inputPivotMaxV, SMART_MOTION_SLOT)
            pivotMaxRPM = inputPivotMaxV
        }
        if(inputPivotMaxA != pivotMaxAcceleration) {
            pivotPID.setSmartMotionMaxAccel(inputPivotMaxA, SMART_MOTION_SLOT)
            pivotMaxAcceleration = inputPivotMaxA
        }
        if(inputPivotMinV != pivotMinRPM) {
            pivotPID.setSmartMotionMinOutputVelocity(inputPivotMinV, SMART_MOTION_SLOT)
            pivotMinRPM = inputPivotMinV
        }
        if(inputPivotAllE != pivotMaxError) {
            pivotPID.setSmartMotionAllowedClosedLoopError(inputPivotAllE, SMART_MOTION_SLOT)
            pivotMaxError = inputPivotAllE
        }

        val pivotSetPoint = SmartDashboard.getNumber("Pivot Set Position", 0.0)
        positionPID.setReference(pivotSetPoint, CANSparkBase.ControlType.kSmartMotion)
        val rotation = pivotEncoder.position

        SmartDashboard.putNumber("Pivot SetPoint", setPoint)
        SmartDashboard.putNumber("Pivot Position", pos)
        SmartDashboard.putNumber("Pivot Motor Output", mainPivotMotor.getAppliedOutput())
        */


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
        private const val ELEVATOR_ANGLE = 28.8309683
        val d2y = sin(ELEVATOR_ANGLE * PI / 180.0)
        val d2x = cos(ELEVATOR_ANGLE * PI / 180.0)
        val MIN_SAFE_ANGLE: Double = TODO()

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
        private var pivotKP = 5e-5
        private var pivotKI = 1e-6
        private var pivotKD = 0.0
        private var pivotIz = 0.0
        private var pivotFF = 0.000156
        private var pivotMin = -1.0
        private var pivotMax = 1.0
        private var pivotMinRPM = 10.0
        private var pivotMaxRPM = 5700.0
        private var pivotMaxAcceleration = 1500.0
        private var pivotMaxError = 5.0
    }
}