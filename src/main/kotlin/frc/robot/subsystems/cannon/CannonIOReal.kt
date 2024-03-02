package frc.robot.subsystems.cannon


import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.constants.CannonConstants

class CannonIOReal : CannonIO {

    val leftShooterMotor: CANSparkMax = CANSparkMax(CannonConstants.LEFT_SHOOTER_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless)
    val rightShooterMotor: CANSparkMax = CANSparkMax(CannonConstants.RIGHT_SHOOTER_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless)

    //TODO: get the actual encoder tick values
    val leftShooterEncoder = leftShooterMotor.getAlternateEncoder(8192);
    val rightShooterEncoder = rightShooterMotor.getAlternateEncoder(8192);

//    val leftShooterEncoder = leftShooterMotor.getEncoder();
//    val rightShooterEncoder = rightShooterMotor.getEncoder();

    val outerIntakeMotor: CANSparkMax = CANSparkMax(CannonConstants.OUTER_INTAKE_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless)
    val innerIntakeMotor: CANSparkMax = CANSparkMax(CannonConstants.INNER_INTAKE_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless)

    val entryBeamBreak: DigitalInput = DigitalInput(4)
    val loadedBeamBreak: DigitalInput = DigitalInput(3)

    val leftShooterPID = leftShooterMotor.pidController
    val rightShooterPID = rightShooterMotor.pidController

    init {
//       leftShooterEncoder.setVelocityConversionFactor(1 / (8192.0 * 2))
//        rightShooterEncoder.setVelocityConversionFactor(1 / (8192.0 * 2))


        leftShooterPID.setP(CannonConstants.leftShooterKP)
        leftShooterPID.setI(CannonConstants.leftShooterKI)
        leftShooterPID.setD(CannonConstants.leftShooterKD)
        leftShooterPID.setFF(CannonConstants.leftShooterFF)
        leftShooterPID.setOutputRange(CannonConstants.leftShooterMin, CannonConstants.leftShooterMax)
        leftShooterPID.setFeedbackDevice(leftShooterEncoder)

        rightShooterPID.setP(CannonConstants.rightShooterKP)
        rightShooterPID.setI(CannonConstants.rightShooterKI)
        rightShooterPID.setD(CannonConstants.rightShooterKD)
        rightShooterPID.setFF(CannonConstants.rightShooterFF)
        leftShooterPID.setOutputRange(CannonConstants.rightShooterMin, CannonConstants.rightShooterMax)
        rightShooterPID.setFeedbackDevice(rightShooterEncoder)
    }

    override fun getLeftShooterVel(): Double {
        return leftShooterEncoder.velocity
    }

    override fun getRightShooterVel(): Double {
        return rightShooterEncoder.velocity
    }

    override fun setLeftShooter(vel: Double) {
        SmartDashboard.putNumber("Left shooter setpoint", vel)
        leftShooterPID.setReference(vel, CANSparkBase.ControlType.kVelocity)
    }

    override fun setRightShooter(vel: Double) {
        SmartDashboard.putNumber("Right Shooter setpoint", vel)
        rightShooterPID.setReference(vel, CANSparkBase.ControlType.kVelocity)
    }

    override fun setInnerIntakePercent(percent: Double) {
        innerIntakeMotor.set(percent)
    }
    override fun setOuterIntakePercent(percent: Double) {
        outerIntakeMotor.set(percent)
    }

    override fun getExitBeamBreak(): Boolean {return true
//        TODO("Not yet implemented")
    }

    override fun getEntryBeamBreak(): Boolean {
        return !entryBeamBreak.get()
    }

    override fun getLoadedBeamBreak(): Boolean {
        return !loadedBeamBreak.get()
    }

}
