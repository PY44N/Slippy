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

    var MIN_SAFE_ANGLE: Double = 0.0
    var TARGET_SAFE_ANGLE: Double = 0.0
    var MIN_SAFE_DISTANCE: Double = 0.0
    var ABS_MIN_ANGLE: Double = 0.0
    var ABS_MAX_ANGLE: Double = 0.0
    var TOP_M: Double = 0.0
    var BOTTOM_M: Double = 0.0
    var SPEAKER_POSITION: Double = 0.0
    var AMP_POSITION: Double = 0.0
    var AMP_ANGLE: Double = 0.0
    var INTAKE_POSITION: Double = 0.0
    var INTAKE_ANGLE: Double = 0.0
    var CROSSBAR_BOTTOM: Double = 0.0
    var CROSSBAR_TOP: Double = 0.0
    var STOW_POSITION: Double = 0.0
    var STOW_ANGLE: Double = 0.0

    var TOP_LIMIT_SWITCH_POSITION = 0.75

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