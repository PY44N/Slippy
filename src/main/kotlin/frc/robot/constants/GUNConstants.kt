package frc.robot.constants

import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin

object GUNConstants {
    // someone rename these
    const val POSITION_GEAR_RATIO = 15
    const val ROTATION_GEAR_RATIO = 100
    const val MAX_PIVOT_HEIGHT_M = 0.8
    const val MIN_PIVOT_HEIGHT_M = 0.0
    const val THING_LENGTH_M = 0.6133
    const val MOVER_GEAR_RADIUS_M = 0.0127
    const val MOVER_GEAR_CIRCUMFERENCE_M = MOVER_GEAR_RADIUS_M * 2 * PI
    val gearCircumfrence = 2 * PI * MOVER_GEAR_RADIUS_M
    const val ELEVATOR_ANGLE = 28.8309683
    val d2y = sin(ELEVATOR_ANGLE * PI / 180.0)
    val d2x = cos(ELEVATOR_ANGLE * PI / 180.0)

    //    var MIN_SAFE_ANGLE: Double = TODO()
//    var TARGET_SAFE_ANGLE: Double = TODO()
//    var MIN_SAFE_DISTANCE: Double = TODO()
//    var ABS_MIN_ANGLE: Double = TODO()
//    var ABS_MAX_ANGLE: Double = TODO()
//    var TOP_M: Double = TODO()
//    var BOTTOM_M: Double = TODO()
//    var SPEAKER_POSITION: Double = TODO()
//    var AMP_POSITION: Double = TODO()
//    var AMP_ANGLE: Double = TODO()
//    var INTAKE_POSITION: Double = TODO()
//    var INTAKE_ANGLE: Double = TODO()
//    var CROSSBAR_BOTTOM: Double = TODO()
//    var CROSSBAR_TOP: Double = TODO()
//    var STOW_POSITION: Double = TODO()
//    var STOW_ANGLE: Double = TODO()
    var rotationOffset: Double = 0.0

    const val SMART_MOTION_SLOT = 0

    var positionKP = 5e-5
    var positionKI = 1e-6
    var positionKD = 0.0
    var positionIz = 0.0
    var positionFF = 0.000156
    var positionMax = 1.0
    var positionMin = -1.0
    var positionMinRPM = 10.0
    var positionMaxRPM = 5700.0
    var positionMaxAcceleration = 1500.0
    var positionMaxError = 5.0

    var rotationKP = 5e-5
    var rotationKI = 1e-6
    var rotationKD = 0.0
    var rotationIz = 0.0
    var rotationFF = 0.000156
    var rotationMin = -1.0
    var rotationMax = 1.0
    var rotationMinRPM = 10.0
    var rotationMaxRPM = 5700.0
    var rotationMaxAcceleration = 1500.0
    var rotationMaxError = 5.0
}