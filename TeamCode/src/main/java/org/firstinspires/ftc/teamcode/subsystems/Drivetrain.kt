package org.firstinspires.ftc.teamcode.subsystems

import org.firstinspires.ftc.teamcode.enums.DrivetrainState
import org.firstinspires.ftc.teamcode.hardware.RobotHardware
import org.firstinspires.ftc.teamcode.interfaces.Subsystem
import kotlin.math.cos
import kotlin.math.sin

class Drivetrain(private val hardware: RobotHardware) : Subsystem {
    private var state: DrivetrainState = DrivetrainState.DRIVER_CONTROLLED_FIELD_CENTRIC

    private fun fieldCentricDrive(forward: Double, strafe: Double, rotate: Double, heading: Double) {
        val cosA = cos(heading)
        val sinA = sin(heading)

        val rotatedForward = forward * cosA - strafe * sinA
        val rotatedStrafe = forward * sinA + strafe * cosA

        drive(rotatedForward, rotatedStrafe, rotate)

    }

    private fun drive(forward: Double, strafe: Double, rotate: Double) {
        hardware.frontLeft.set(forward + strafe + rotate)
        hardware.frontRight.set(forward - strafe - rotate)
        hardware.backLeft.set(forward - strafe + rotate)
        hardware.backRight.set(forward + strafe - rotate)
    }

    override fun periodic() {
        when (state) {
            DrivetrainState.DRIVER_CONTROLLED_FIELD_CENTRIC -> {
                val forward = -hardware.gamepad.leftY
                val strafe = hardware.gamepad.leftX
                val rotate = hardware.gamepad.rightX
                drive(forward, strafe, rotate)
            }

            DrivetrainState.DRIVER_CONTROLLED_ROBOT_CENTRIC -> {
                val forward = -hardware.gamepad.leftY
                val strafe = hardware.gamepad.leftX
                val rotate = hardware.gamepad.rightX
                fieldCentricDrive(forward, strafe, rotate, 0.0) // Replace 0.0 with actual heading from IMU
            }
        }
    }
}