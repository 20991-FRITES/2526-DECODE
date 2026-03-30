package config

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import math.Distance
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

@Configurable
object HardwareConfig {
    var FRONT_LEFT_MOTOR_ID: String = "front_left"
    var FRONT_RIGHT_MOTOR_ID: String = "front_right"
    var BACK_LEFT_MOTOR_ID: String = "back_left"
    var BACK_RIGHT_MOTOR_ID: String = "back_right"
    var FRONT_LEFT_REVERSED: Boolean = true
    var FRONT_RIGHT_REVERSED: Boolean = false
    var BACK_LEFT_REVERSED: Boolean = true
    var BACK_RIGHT_REVERSED: Boolean = false

    var CANNON_MOTOR_LEFT_ID: String = "cannon_motor_left"
    var CANNON_MOTOR_RIGHT_ID: String = "cannon_motor_right"
    var CANNON_BUFFER_RIGHT: String = "cannon_buffer_right"
    var CANNON_BUFFER_LEFT: String = "cannon_buffer_left"
    var INTAKE_MOTOR_ID: String = "intake_motor"
    const val INTAKE_SWITCHER_SERVO: String = "intake_switcher_servo"
    const val LIMELIGHT_CAMERA_ID: String = "limelight"
    const val ODOMETRY_POD_ID: String = "pinpoint"
    var IMU_ID: String = "imu"

    const val DISTANCE_SENSOR_LEFT_ID: String = "distance_left"
    const val DISTANCE_SENSOR_RIGHT_ID: String = "distance_right"
    const val DISTANCE_SENSOR_INTAKE_LEFT_ID: String = "distance_intake_left"
    const val DISTANCE_SENSOR_INTAKE_RIGHT_ID: String = "distance_intake_right"
    const val NEOPIXEL_ID: String = "neopixel"

    var SHOOTER_MAX_VELOCITY: Double = 2450.0

    var ROBOT_SIZE = Distance(DistanceUnit.INCH, 18.0)

    @JvmField
    var PINPOINT_OFFSET_X = Distance(DistanceUnit.MM, 94.0)
    @JvmField
    var PINPOINT_OFFSET_Y = Distance(DistanceUnit.MM, -155.0)

    var PINPOINT_X_DIRECTION = GoBildaPinpointDriver.EncoderDirection.REVERSED
    var PINPOINT_Y_DIRECTION = GoBildaPinpointDriver.EncoderDirection.REVERSED
}