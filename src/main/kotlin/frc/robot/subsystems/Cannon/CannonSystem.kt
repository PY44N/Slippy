package frc.robot.subsystems.Cannon

import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.constants.CannonConstants
import frc.robot.subsystems.Cannon.CannonIO

class CannonSystem(val io: CannonIO) : SubsystemBase() {

    var desiredLeftVel = 0.0
    var desiredRightVel = 0.0


    init {

    }

    fun prep(leftVel: Double, rightVel: Double) {
        desiredLeftVel = leftVel;
        desiredRightVel = rightVel;

        io.setLeftShooterVel(desiredLeftVel)
        io.setRightShooterVel(desiredRightVel)
    }

    fun shoot(leftVel: Double, rightVel: Double) {
        desiredLeftVel = leftVel;
        desiredRightVel = rightVel;

        io.setLeftShooterVel(desiredLeftVel)
        io.setRightShooterVel(desiredRightVel)
    }

    fun intake() {
        io.setInnerIntakePercent(CannonConstants.innerIntakePercent)
        io.setOuterIntakePercent(CannonConstants.outerIntakePercent)
    }

    override fun periodic() {

    }

    override fun simulationPeriodic() {

    }
}
