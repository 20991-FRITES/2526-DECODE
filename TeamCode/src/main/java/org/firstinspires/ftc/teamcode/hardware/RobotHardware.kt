package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.hardware.motors.CRServo
import com.seattlesolvers.solverslib.hardware.motors.MotorEx
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

    fun init(hardwareMap: HardwareMap, gamepad1: Gamepad) {
        frontLeft = MotorEx(hardwareMap, "front_left")
        frontRight = MotorEx(hardwareMap, "front_right")
        backLeft = MotorEx(hardwareMap, "back_left")
        backRight = MotorEx(hardwareMap, "back_right")

        // Set motor directions for drivetrain
        frontRight.setInverted(true)
        backRight.setInverted(true)

        intake = MotorEx(hardwareMap, "intake")

        flywheelLeft = MotorEx(hardwareMap, "shooter_left")
        flywheelRight = MotorEx(hardwareMap, "shooter_right")

        flywheelRight.setInverted(true)

        bufferLeft = CRServo(hardwareMap, "buffer_left")
        bufferRight = CRServo(hardwareMap, "buffer_right")

        gamepad = GamepadEx(gamepad1)

        limelight = hardwareMap.get(Limelight3A::class.java, "limelight")
        pinpoint = hardwareMap.get(GoBildaPinpointDriver::class.java, "pinpoint")

        pinpoint.setOffsets(0.0, 0.0, DistanceUnit.MM)
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD)

        pinpoint.resetPosAndIMU();
    }
}