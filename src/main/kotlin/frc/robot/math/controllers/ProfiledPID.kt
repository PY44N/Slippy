package frc.robot.util

import cshcyberhawks.lib.math.Timer
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard

class ProfiledPID(val p: Double, val i: Double, val d: Double, var trapConstraints: TrapezoidProfile.Constraints) :
    Sendable {
    val pidController = PIDController(p, i, d)
    var trapezoidProfile = TrapezoidProfile(trapConstraints)
    var desiredTrapState = TrapezoidProfile.State()
    var currentTrapState = TrapezoidProfile.State()
    private var lastMeasurement = 0.0

    private var minimumInput = 0.0
    private var maximumInput = 0.0
    private val trapTimer = Timer()

    var goal = 0.0
        set(value) {
            desiredTrapState = TrapezoidProfile.State(value, 0.0)
            currentTrapState = TrapezoidProfile.State(value, 0.0)
            trapTimer.reset()
            trapTimer.start()
            field = value
        }

    fun enableContinuousInput(min: Double, max: Double) {
        pidController.enableContinuousInput(min, max)
        minimumInput = min
        maximumInput = max
    }

    fun calculate(measured: Double): Double {
        if (pidController.isContinuousInputEnabled) {
            // Get error which is the smallest distance between goal and measurement
            val errorBound = (maximumInput - minimumInput) / 2.0;
            val goalMinDistance =
                MathUtil.inputModulus(desiredTrapState.position - measured, -errorBound, errorBound);
            val setpointMinDistance =
                MathUtil.inputModulus(currentTrapState.position - measured, -errorBound, errorBound);

            // Recompute the profile goal with the smallest error, thus giving the shortest path. The goal
            // may be outside the input range after this operation, but that's OK because the controller
            // will still go there and report an error of zero. In other words, the setpoint only needs to
            // be offset from the measurement by the input range modulus; they don't need to be equal.
            desiredTrapState.position = goalMinDistance + measured;
            currentTrapState.position = setpointMinDistance + measured;
        }

        lastMeasurement = measured
        currentTrapState = trapezoidProfile.calculate(pidController.period, currentTrapState, desiredTrapState)

        SmartDashboard.putNumber("ProfiledPID Period", pidController.period)

        return pidController.calculate(measured, currentTrapState.position)
    }

    fun reset(measuredPosition: Double, measuredVelocity: Double = 0.0) {
        pidController.reset();
        trapTimer.reset()
        trapTimer.start()
        currentTrapState = TrapezoidProfile.State(measuredPosition, measuredVelocity)
    }

    fun logStates(name: String) {
        SmartDashboard.putNumber("${name} Position Error", pidController.positionError)
        SmartDashboard.putNumber("${name} Position", lastMeasurement)
        SmartDashboard.putNumber("${name} Current Desired Position", currentTrapState.position)
    }

    override fun initSendable(builder: SendableBuilder) {
        builder.setSmartDashboardType("ProfiledPID")
        builder.addDoubleProperty("p", pidController::getP, pidController::setP)
        builder.addDoubleProperty("i", pidController::getI, pidController::setI)
        builder.addDoubleProperty("d", pidController::getD, pidController::setD)
        builder.addDoubleProperty(
            "Acceleration",
            { trapConstraints.maxAcceleration },
            {
                trapConstraints = TrapezoidProfile.Constraints(trapConstraints.maxVelocity, it)
                trapezoidProfile = TrapezoidProfile(trapConstraints)
            })

        builder.addDoubleProperty(
            "Velocity",
            { trapConstraints.maxVelocity },
            {
                trapConstraints = TrapezoidProfile.Constraints(it, trapConstraints.maxAcceleration)
                trapezoidProfile = TrapezoidProfile(trapConstraints)
            })
        builder.addDoubleProperty("goal", { goal }, { goal = it })
    }
}