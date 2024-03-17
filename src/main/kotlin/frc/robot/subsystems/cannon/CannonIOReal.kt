package frc.robot.subsystems.cannon

import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.Encoder
import frc.robot.constants.CannonConstants

class CannonIOReal : CannonIO {

    val leftShooterMotor: CANSparkMax =
            CANSparkMax(CannonConstants.LEFT_SHOOTER_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless)
    val rightShooterMotor: CANSparkMax =
            CANSparkMax(CannonConstants.RIGHT_SHOOTER_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless)

//    val leftShooterEncoder = leftShooterMotor.getAlternateEncoder(8192);
//    val rightShooterEncoder = rightShooterMotor.getAlternateEncoder(8192);

    val leftShooterMotorEncoder = leftShooterMotor.getEncoder();
    val rightShooterMotorEncoder = rightShooterMotor.getEncoder();

    val rightShooterEncoder = Encoder(8, 7)
    val leftShooterEncoder = Encoder(6, 5)

    val outerIntakeMotor: CANSparkMax =
            CANSparkMax(CannonConstants.OUTER_INTAKE_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless)
    val innerIntakeMotor: CANSparkMax =
            CANSparkMax(CannonConstants.INNER_INTAKE_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless)

    val entryBeamBreak: DigitalInput = DigitalInput(3)
    val loadedBeamBreak: DigitalInput = DigitalInput(2)

    init {
//       leftShooterEncoder.setVelocityConversionFactor(1 / (8192.0 * (27/40)))
//        rightShooterEncoder.setVelocityConversionFactor(1 / (8192.0 * (27/40)))

//        leftShooterPID.setP(CannonConstants.leftShooterKP)
//        leftShooterPID.setI(CannonConstants.leftShooterKI)
//        leftShooterPID.setD(CannonConstants.leftShooterKD)
//        leftShooterPID.setFF(CannonConstants.leftShooterFF)
//        leftShooterPID.setOutputRange(CannonConstants.leftShooterMin, CannonConstants.leftShooterMax)
//        leftShooterPID.setFeedbackDevice(leftShooterEncoder)
//
//        rightShooterPID.setP(CannonConstants.rightShooterKP)
//        rightShooterPID.setI(CannonConstants.rightShooterKI)
//        rightShooterPID.setD(CannonConstants.rightShooterKD)
//        rightShooterPID.setFF(CannonConstants.rightShooterFF)
//        leftShooterPID.setOutputRange(CannonConstants.rightShooterMin, CannonConstants.rightShooterMax)
//        rightShooterPID.setFeedbackDevice(rightShooterEncoder)

//        leftShooterMotor.restoreFactoryDefaults()
//        rightShooterMotor.restoreFactoryDefaults()
        rightShooterMotor.inverted = true
//        leftShooterEncoder.setVelocityConversionFactor(1 / 8192.0)
//        rightShooterEncoder.setVelocityConversionFactor(1 / 8192.0)

        rightShooterEncoder.distancePerPulse = 60.0 / 8192.0 * 4
        leftShooterEncoder.distancePerPulse = 60.0 / 8192.0 * 4
    }

    override fun getLeftShooterVel(): Double {

//        return leftShooterMotorEncoder.velocity * (40.0 / 27.0)
        return -leftShooterEncoder.rate
    }

    override fun getLeftShooterTBVel(): Double {
        return leftShooterEncoder.rate

    }

    override fun getRightShooterTBVel(): Double {
        return rightShooterEncoder.rate

    }

    override fun getRightShooterVel(): Double {
//        return rightShooterMotorEncoder.velocity * (40.0 / 27.0)
        return rightShooterEncoder.rate
    }

    override fun setLeftShooter(percent: Double) {
        leftShooterMotor.set(percent)
    }

    override fun setRightShooter(percent: Double) {
        rightShooterMotor.set(percent)
    }

    override fun setInnerIntakePercent(percent: Double) {
        innerIntakeMotor.set(percent)
    }

    override fun setOuterIntakePercent(percent: Double) {
        outerIntakeMotor.set(percent)
    }

    override fun getExitBeamBreak(): Boolean {
        return true
//        TODO("Not yet implemented")
    }

    override fun getEntryBeamBreak(): Boolean {
        return !entryBeamBreak.get()
    }

    override fun getLoadedBeamBreak(): Boolean {
        return !loadedBeamBreak.get()
    }

    override fun getIntakePosition(): Double {
        return innerIntakeMotor.encoder.position
    }

}
