package frc.robot.subsystems.Cannon


import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import frc.robot.constants.CannonConstants

class CannonIOReal : CannonIO {

    val leftShooterMotor: CANSparkMax = CANSparkMax(CannonConstants.LEFT_SHOOTER_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless)
    val rightShooterMotor: CANSparkMax = CANSparkMax(CannonConstants.RIGHT_SHOOTER_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless)

    val leftShooterPID = leftShooterMotor.pidController
    val rightShooterPID = rightShooterMotor.pidController

    //TODO: get the actual encoder tick values
    val leftShooterEncoder = leftShooterMotor.getAlternateEncoder(0);
    val rightShooterEncoder = rightShooterMotor.getAlternateEncoder(0);

    val outerIntakeMotor: CANSparkMax = CANSparkMax(CannonConstants.OUTER_INTAKE_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless)
    val innerIntakeMotor: CANSparkMax = CANSparkMax(CannonConstants.INNER_INTAKE_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless)

    init {
        leftShooterPID.setP(CannonConstants.leftShooterKP)
        leftShooterPID.setI(CannonConstants.leftShooterKI)
        leftShooterPID.setD(CannonConstants.leftShooterKD)
        leftShooterPID.setIZone(CannonConstants.leftShooterIz)
        leftShooterPID.setFF(CannonConstants.leftShooterFF)

        rightShooterPID.setP(CannonConstants.rightShooterKP)
        rightShooterPID.setI(CannonConstants.rightShooterKI)
        rightShooterPID.setD(CannonConstants.rightShooterKD)
        rightShooterPID.setIZone(CannonConstants.rightShooterIz)
        rightShooterPID.setFF(CannonConstants.rightShooterFF)
    }

    override fun getLeftShooterVel(): Double {
        return leftShooterEncoder.velocity
    }

    override fun getRightShooterVel(): Double {
        return rightShooterEncoder.velocity
    }



    override fun setLeftShooterVel(vel: Double) {
        leftShooterPID.setReference(vel, CANSparkBase.ControlType.kVelocity)
    }

    override fun setRightShooterVel(vel: Double) {
        rightShooterPID.setReference(vel, CANSparkBase.ControlType.kVelocity)
    }

    override fun setInnerIntakePercent(percent: Double) {
        innerIntakeMotor.set(percent)
    }
    override fun setOuterIntakePercent(percent: Double) {
        outerIntakeMotor.set(percent)
    }

    override fun getExitBeamBreak(): Boolean {
        TODO("Not yet implemented")
    }

    override fun getEntryBeamBreak(): Boolean {
        TODO("Not yet implemented")
    }

    override fun getLoadedBeamBreak(): Boolean {
        TODO("Not yet implemented")
    }

}
