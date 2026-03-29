package org.firstinspires.ftc.teamcode.subsystems

import com.seattlesolvers.solverslib.controller.PIDFController
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.teamcode.FieldConfig
import org.firstinspires.ftc.teamcode.enums.DrivetrainState
import org.firstinspires.ftc.teamcode.hardware.RobotHardware
import org.firstinspires.ftc.teamcode.interfaces.Subsystem
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin

class Drivetrain(private val hardware: RobotHardware) : Subsystem {
    private var state: DrivetrainState = DrivetrainState.DRIVER_CONTROLLED_FIELD_CENTRIC

    private val rotationPIDF = PIDFController(0.1, 0.01, 0.05, 0.0)

    private fun fieldCentricDrive(
        forward: Double,
        strafe: Double,
        rotate: Double,
        heading: Double
    ) {
        val cosA = cos(heading)
        val sinA = sin(heading)

        val rotatedForward = forward * cosA - strafe * sinA
        val rotatedStrafe = forward * sinA + strafe * cosA

        drive(rotatedForward, rotatedStrafe, rotate)

    }

    private fun assistedRotation(
        forward: Double,
        strafe: Double,
        targetHeading: Double,
        currentHeading: Double
    ) {
        val rotationPower = rotationPIDF.calculate(currentHeading, targetHeading)
        fieldCentricDrive(forward, strafe, rotationPower, currentHeading)
    }

    private fun drive(forward: Double, strafe: Double, rotate: Double) {
        hardware.frontLeft.set(forward + strafe + rotate)
        hardware.frontRight.set(forward - strafe - rotate)
        hardware.backLeft.set(forward - strafe + rotate)
        hardware.backRight.set(forward + strafe - rotate)
    }

    override fun periodic(botpose: Pose2D) {
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
                fieldCentricDrive(
                    forward,
                    strafe,
                    rotate,
                    botpose.getHeading(AngleUnit.RADIANS)
                )
            }

            DrivetrainState.LOCK_TOWARDS_GOAL -> {
                val forward = -hardware.gamepad.leftY
                val strafe = hardware.gamepad.leftX
                val currentHeading: Double = botpose.getHeading(AngleUnit.RADIANS)
                val currentX = botpose.getX(DistanceUnit.CM)
                val currentY = botpose.getY(DistanceUnit.CM)
                val goalPos = FieldConfig.redGoalPose;
                val targetHeading = atan2(
                    goalPos.getY(DistanceUnit.CM) - currentY,
                    goalPos.getX(DistanceUnit.CM) - currentX
                )
                assistedRotation(forward, strafe, targetHeading, currentHeading)
            }
        }
    }
}