package org.firstinspires.ftc.teamcode.subsystems

import com.seattlesolvers.solverslib.controller.PIDFController
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.teamcode.config.FieldConfig
import org.firstinspires.ftc.teamcode.config.FieldConfig.mirror
import org.firstinspires.ftc.teamcode.enums.DrivetrainState
import org.firstinspires.ftc.teamcode.enums.Team
import org.firstinspires.ftc.teamcode.hardware.RobotHardware
import org.firstinspires.ftc.teamcode.interfaces.Subsystem
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin

class Drivetrain(private val hardware: RobotHardware, private val team: Team) : Subsystem {
    private var state: DrivetrainState = DrivetrainState.DRIVER_CONTROLLED_FIELD_CENTRIC

    private val rotationPIDF = PIDFController(0.85, 0.0, 0.001, 0.025)
    private val forwardPIDF = PIDFController(0.05, 0.0, 0.00001, 0.025)
    private val strafePIDF = PIDFController(0.075, 0.0, 0.001, 0.025)

    fun changeState(newState: DrivetrainState) {
        state = newState
    }

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

    private fun moveToPose(targetPose: Pose2D, currentPose: Pose2D): Boolean {
        // Get current position
        val currentX = currentPose.getX(DistanceUnit.MM)
        val currentY = currentPose.getY(DistanceUnit.MM)
        val currentHeading = currentPose.getHeading(AngleUnit.RADIANS)

        // Get target position
        val targetX = targetPose.getX(DistanceUnit.MM)
        val targetY = targetPose.getY(DistanceUnit.MM)
        val targetHeading = targetPose.getHeading(AngleUnit.RADIANS)

        // Field-centric error
        val xError = targetX - currentX
        val yError = targetY - currentY
        var headingError = targetHeading - currentHeading

        // Normalize heading error to [-pi, pi]
        while (headingError > Math.PI) headingError -= 2 * Math.PI
        while (headingError < -Math.PI) headingError += 2 * Math.PI

        // Rotate field error into robot frame
        val cos = cos(currentHeading)
        val sin = sin(currentHeading)

        val forwardError = xError * cos + yError * sin
        val strafeError = -xError * sin + yError * cos

        // PID outputs
        val forwardPower = forwardPIDF.calculate(forwardError)
        val strafePower = strafePIDF.calculate(strafeError)
        val rotationPower = rotationPIDF.calculate(headingError)

        // Normalize motor powers
        val maxPower = maxOf(
            abs(forwardPower),
            abs(strafePower),
            abs(rotationPower),
            1.0
        )

        drive(
            forwardPower / maxPower,
            strafePower / maxPower,
            rotationPower / maxPower
        )

        // Completion thresholds
        val positionThreshold = 1.0 // inches
        val headingThreshold = Math.toRadians(3.0)

        val distance = Math.hypot(xError, yError)

        return distance < positionThreshold && Math.abs(headingError) < headingThreshold
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
                val goalPos = mirror(FieldConfig.redGoalPose, team);
                val targetHeading = atan2(
                    goalPos.getY(DistanceUnit.CM) - currentY,
                    goalPos.getX(DistanceUnit.CM) - currentX
                )
                assistedRotation(forward, strafe, targetHeading, currentHeading)
            }

            DrivetrainState.MACRO_MOVE_TO_SHOOT -> {
                if (moveToPose(mirror(FieldConfig.redShootingPose, team), botpose)) {
                    changeState(DrivetrainState.DRIVER_CONTROLLED_FIELD_CENTRIC)
                }
            }

            DrivetrainState.MACRO_MOVE_TO_INTAKE -> {
                if (moveToPose(mirror(FieldConfig.redIntakingPose, team), botpose)) {
                    changeState(DrivetrainState.DRIVER_CONTROLLED_FIELD_CENTRIC)
                }
            }
        }
    }
}