package frc.robot.subsystems

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.SubsystemBase
import au.edu.federation.caliko.FabrikStructure2D
import au.edu.federation.caliko.FabrikChain2D
import au.edu.federation.caliko.FabrikBone2D



class TrunkSystem : SubsystemBase() {

    init {
        val structure: FabrikStructure2D = FabrikStructure2D()
        val chain: FabrikChain2D = FabrikChain2D()

        val baseBone: FabrikBone2D = FabrikBone2D(0.0f, 0.0f, 0.0f, 0.0f)
        baseBone.setJoint()


        chain.addBone(baseBone)

        val endBone: FabrikBone2D = FabrikBone2D(0.0f, 0.0f, 0.0f, 0.0f)
        chain.addBone(endBone)



        structure.addChain(chain)
    }

    fun setDesiredPosition(angle: Rotation2d, distance: Double) {}

    // fun setDesiredSpeedRPM(left: Double, right: Double) {
    //
    // }
    //
    // fun setDesiredSpeedPower(left: Double, right: Double) {
    //
    // }
    //
    // fun shoot(leftPower: Double, rightPower: Double, angle: Rotation2d, distance: Double) {
    //
    // }

    override fun periodic() {}

    override fun simulationPeriodic() {}
}
