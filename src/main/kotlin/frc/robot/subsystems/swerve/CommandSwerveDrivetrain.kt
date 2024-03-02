package frc.robot.subsystems.swerve

import com.ctre.phoenix6.Utils
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest
import edu.wpi.first.wpilibj.Notifier
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import java.util.function.Supplier

class CommandSwerveDrivetrain : SwerveDrivetrain, Subsystem {
    private lateinit var m_simNotifier: Notifier
    private var m_lastSimTime = 0.0

    constructor(
            driveTrainConstants: SwerveDrivetrainConstants?,
            OdometryUpdateFrequency: Double,
            vararg modules: SwerveModuleConstants?
    ) : super(driveTrainConstants, OdometryUpdateFrequency, *modules) {
        if (Utils.isSimulation()) {
            startSimThread()
        }
    }

    constructor(driveTrainConstants: SwerveDrivetrainConstants?, vararg modules: SwerveModuleConstants?) : super(
            driveTrainConstants,
            *modules
    ) {
        if (Utils.isSimulation()) {
            startSimThread()
        }
    }

    fun applyRequest(requestSupplier: Supplier<SwerveRequest?>): Command {
        return run { this.setControl(requestSupplier.get()) }
    }

    private fun startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds()

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = Notifier {
            val currentTime = Utils.getCurrentTimeSeconds()
            val deltaTime = currentTime - m_lastSimTime
            m_lastSimTime = currentTime

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage())
        }
        m_simNotifier!!.startPeriodic(kSimLoopPeriod)
    }

    companion object {
        private const val kSimLoopPeriod = 0.005 // 5 ms
    }
}
