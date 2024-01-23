package frc.robot.util

import com.ctre.phoenix6.configs.Pigeon2Configuration
import com.ctre.phoenix6.configs.Pigeon2Configurator
import com.ctre.phoenix6.hardware.Pigeon2
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Translation3d
import swervelib.imu.SwerveIMU
import java.util.*

class DualPigeon2Swerve(canidNormal: Int, canidReverse: Int, canbusName: String) : SwerveIMU() {
    val normalPigeon: Pigeon2
    val reversePigeon: Pigeon2
    init {
        normalPigeon = Pigeon2(canidNormal, canbusName)
        reversePigeon = Pigeon2(canidReverse, canbusName)
    }

    override fun factoryDefault() {
        val cfgN: Pigeon2Configurator = normalPigeon.configurator
        val cfgR: Pigeon2Configurator = reversePigeon.configurator

        val config = Pigeon2Configuration()

        cfgN.apply(config.Pigeon2Features.withEnableCompass(false))
        cfgR.apply(config.Pigeon2Features.withEnableCompass(false))
    }

    override fun clearStickyFaults() {
        normalPigeon.clearStickyFaults()
        reversePigeon.clearStickyFaults()
    }

    override fun setOffset(offset: Rotation3d?) {

    }

    override fun getRawRotation3d(): Rotation3d {
        TODO("Not yet implemented")
    }

    override fun getRotation3d(): Rotation3d {
        TODO("Not yet implemented")
    }

    override fun getAccel(): Optional<Translation3d> {
        TODO("Not yet implemented")
    }

    override fun getIMU(): Any {
        TODO("Not yet implemented")
    }
}