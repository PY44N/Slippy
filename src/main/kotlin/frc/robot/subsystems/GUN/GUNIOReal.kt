package frc.robot.subsystems.GUN

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import frc.robot.constants.GunConstants

class GUNIOReal : GUNIO {

    val leftMotor: CANSparkMax = CANSparkMax(GunConstants.LEFT_SHOOTER_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless)
    val rightMotor: CANSparkMax = CANSparkMax(GunConstants.RIGHT_SHOOTER_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless)

    val leftPID = leftMotor.pidController
    val rightPID = rightMotor.pidController

    val leftEncoder = leftMotor.getAlternateEncoder(00);

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

    override fun getLeftSpeed(): Double {
        return leftMotor.get()
    }

    override fun getRightSpeed(): Double {
        TODO("Not yet implemented")
    }

    override fun getExitBeamBreak(): Boolean {
        TODO("Not yet implemented")
    }

    override fun setLeftSpeed(speed: Double) {
        leftPID.setReference(speed, CANSparkBase.ControlType.kVelocity)
    }

    override fun setRightSpeed(speed: Double) {
        rightPID.setReference(speed, CANSparkBase.ControlType.kVelocity)
    }
}