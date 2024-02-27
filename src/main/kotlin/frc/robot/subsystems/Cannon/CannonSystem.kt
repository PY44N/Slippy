package frc.robot.subsystems.Cannon

import edu.wpi.first.wpilibj2.command.SubsystemBase

class CannonSystem : SubsystemBase() {

    var desiredLeftVel = 0.0
    var desiredRightVel = 0.0


    init {

    }

    fun prep(leftVel: Double, rightVel: Double) {
        desiredLeftVel = leftVel;
        desiredRightVel = rightVel;
    }

    fun shoot(leftVel: Double, rightVel: Double) {

    }

    override fun periodic() {

    }

    override fun simulationPeriodic() {

    }
}
