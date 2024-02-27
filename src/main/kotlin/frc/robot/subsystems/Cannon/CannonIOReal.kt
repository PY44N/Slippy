package frc.robot.subsystems.Cannon


import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import frc.robot.constants.GunConstants

class CannonIOReal : CannonIO {

    val leftMotor: CANSparkMax = CANSparkMax(GunConstants.LEFT_SHOOTER_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless)
    val rightMotor: CANSparkMax = CANSparkMax(GunConstants.RIGHT_SHOOTER_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless)

    val leftPID = leftMotor.pidController
    val rightPID = rightMotor.pidController

    //TODO: get the actual encoder tick values
    val leftEncoder = leftMotor.getAlternateEncoder(0);
    val rightEncoder = rightMotor.getAlternateEncoder(0);

    init {
        leftPID.setP(GunConstants.leftKP)
        leftPID.setI(GunConstants.leftKI)
        leftPID.setD(GunConstants.leftKD)
        leftPID.setIZone(GunConstants.leftIz)
        leftPID.setFF(GunConstants.leftFF)

        rightPID.setP(GunConstants.rightKP)
        rightPID.setI(GunConstants.rightKI)
        rightPID.setD(GunConstants.rightKD)
        rightPID.setIZone(GunConstants.rightIz)
        rightPID.setFF(GunConstants.rightFF)
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
