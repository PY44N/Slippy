package frc.robot.subsystems.Cannon


import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import frc.robot.constants.CannonConstants

class CannonIOReal : CannonIO {

    val leftMotor: CANSparkMax = CANSparkMax(CannonConstants.LEFT_SHOOTER_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless)
    val rightMotor: CANSparkMax = CANSparkMax(CannonConstants.RIGHT_SHOOTER_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless)

    val leftPID = leftMotor.pidController
    val rightPID = rightMotor.pidController

    //TODO: get the actual encoder tick values
    val leftEncoder = leftMotor.getAlternateEncoder(0);
    val rightEncoder = rightMotor.getAlternateEncoder(0);

    init {
        leftPID.setP(CannonConstants.leftKP)
        leftPID.setI(CannonConstants.leftKI)
        leftPID.setD(CannonConstants.leftKD)
        leftPID.setIZone(CannonConstants.leftIz)
        leftPID.setFF(CannonConstants.leftFF)

        rightPID.setP(CannonConstants.rightKP)
        rightPID.setI(CannonConstants.rightKI)
        rightPID.setD(CannonConstants.rightKD)
        rightPID.setIZone(CannonConstants.rightIz)
        rightPID.setFF(CannonConstants.rightFF)
    }

    override fun getLeftVel(): Double {
        return leftEncoder.velocity
    }

    override fun getRightVel(): Double {
        return rightEncoder.velocity
    }

    override fun getExitBeamBreak(): Boolean {
        TODO("Not yet implemented")
    }

    override fun setLeftVel(vel: Double) {
        leftPID.setReference(vel, CANSparkBase.ControlType.kVelocity)
    }

    override fun setRightVel(vel: Double) {
        rightPID.setReference(vel, CANSparkBase.ControlType.kVelocity)
    }
}
