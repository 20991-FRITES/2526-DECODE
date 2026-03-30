package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.hardware.motors.CRServo
import com.seattlesolvers.solverslib.hardware.motors.MotorEx
import config.HardwareConfig
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

class RobotHardware {
    lateinit var frontLeft: MotorEx
    lateinit var frontRight: MotorEx
    lateinit var backLeft: MotorEx
    lateinit var backRight: MotorEx

    lateinit var intake: MotorEx

    lateinit var flywheelLeft: MotorEx
    lateinit var flywheelRight: MotorEx

    lateinit var bufferLeft: CRServo
    lateinit var bufferRight: CRServo

    lateinit var gamepad: GamepadEx

    lateinit var limelight: Limelight3A
    lateinit var pinpoint: GoBildaPinpointDriver

    lateinit var distanceLeft: DistanceSensor
    lateinit var distanceRight: DistanceSensor
    lateinit var distanceIntakeLeft: DistanceSensor
    lateinit var distanceIntakeRight: DistanceSensor

    fun init(hardwareMap: HardwareMap, gamepad1: Gamepad) {
        frontLeft = MotorEx(hardwareMap, HardwareConfig.FRONT_LEFT_MOTOR_ID)
        frontRight = MotorEx(hardwareMap, HardwareConfig.FRONT_RIGHT_MOTOR_ID)
        backLeft = MotorEx(hardwareMap, HardwareConfig.BACK_LEFT_MOTOR_ID)
        backRight = MotorEx(hardwareMap, HardwareConfig.BACK_RIGHT_MOTOR_ID)

        frontLeft.setInverted(HardwareConfig.FRONT_LEFT_REVERSED)
        frontRight.setInverted(HardwareConfig.FRONT_RIGHT_REVERSED)
        backLeft.setInverted(HardwareConfig.BACK_LEFT_REVERSED)
        backRight.setInverted(HardwareConfig.BACK_RIGHT_REVERSED)

        intake = MotorEx(hardwareMap, HardwareConfig.INTAKE_MOTOR_ID)

        flywheelLeft = MotorEx(hardwareMap, HardwareConfig.CANNON_MOTOR_LEFT_ID)
        flywheelRight = MotorEx(hardwareMap, HardwareConfig.CANNON_MOTOR_RIGHT_ID)

        flywheelRight.setInverted(true)

        bufferLeft = CRServo(hardwareMap, HardwareConfig.CANNON_BUFFER_LEFT)
        bufferRight = CRServo(hardwareMap, HardwareConfig.CANNON_BUFFER_RIGHT)

        gamepad = GamepadEx(gamepad1)

        limelight = hardwareMap.get(Limelight3A::class.java, HardwareConfig.LIMELIGHT_CAMERA_ID)
        pinpoint =
            hardwareMap.get(GoBildaPinpointDriver::class.java, HardwareConfig.ODOMETRY_POD_ID)

        pinpoint.setOffsets(
            HardwareConfig.PINPOINT_OFFSET_X.toMillimeters(),
            HardwareConfig.PINPOINT_OFFSET_Y.toMillimeters(),
            DistanceUnit.MM
        )
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        pinpoint.setEncoderDirections(
            HardwareConfig.PINPOINT_X_DIRECTION,
            HardwareConfig.PINPOINT_Y_DIRECTION
        )

        pinpoint.resetPosAndIMU();

        distanceLeft =
            hardwareMap.get(DistanceSensor::class.java, HardwareConfig.DISTANCE_SENSOR_LEFT_ID)
        distanceRight =
            hardwareMap.get(DistanceSensor::class.java, HardwareConfig.DISTANCE_SENSOR_RIGHT_ID)
        distanceIntakeLeft = hardwareMap.get(
            DistanceSensor::class.java,
            HardwareConfig.DISTANCE_SENSOR_INTAKE_LEFT_ID
        )
        distanceIntakeRight = hardwareMap.get(
            DistanceSensor::class.java,
            HardwareConfig.DISTANCE_SENSOR_INTAKE_RIGHT_ID
        )
    }
}