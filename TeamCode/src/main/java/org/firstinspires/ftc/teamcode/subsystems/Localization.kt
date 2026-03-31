package org.firstinspires.ftc.teamcode.subsystems

import com.bylazar.field.PanelsField
import com.pedropathing.ftc.FTCCoordinates
import com.pedropathing.ftc.InvertedFTCCoordinates
import com.pedropathing.ftc.PoseConverter
import com.qualcomm.hardware.limelightvision.LLResult
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.teamcode.BotContext
import org.firstinspires.ftc.teamcode.config.AutoPaths
import org.firstinspires.ftc.teamcode.enums.LocalizationState
import org.firstinspires.ftc.teamcode.hardware.RobotHardware
import org.firstinspires.ftc.teamcode.interfaces.Subsystem
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin

class Localization(private val hardware: RobotHardware) : Subsystem {
    private var state: LocalizationState = LocalizationState.FUSED

    private var pose: Pose2D = PoseConverter.poseToPose2D(
        AutoPaths.parkPose,
        InvertedFTCCoordinates.INSTANCE
    ) // Teleop start pose
    val panelsField = PanelsField.field

    init {
        panelsField.setOffsets(PanelsField.presets.DEFAULT_FTC)
    }

    fun drawRobot(robotX: Double, robotY: Double, heading: Double) {
        val size = 18.0
        val half = size / 2.0

        fun rotate(x: Double, y: Double, angle: Double): Pair<Double, Double> {
            val cosA = cos(angle)
            val sinA = sin(angle)
            val rx = x * cosA - y * sinA
            val ry = x * sinA + y * cosA
            return Pair(rx, ry)
        }

        // Robot square corners (relative to center)
        val corners = listOf(
            Pair(-half, -half),
            Pair(half, -half),
            Pair(half, half),
            Pair(-half, half)
        )

        // Draw square
        for (i in corners.indices) {
            val p1 = rotate(corners[i].first, corners[i].second, heading)
            val p2 = rotate(
                corners[(i + 1) % corners.size].first,
                corners[(i + 1) % corners.size].second,
                heading
            )

            panelsField.moveCursor(robotX + p1.first, robotY + p1.second)
            panelsField.line(
                x2 = robotX + p2.first,
                y2 = robotY + p2.second
            )
        }

        // Draw forward arrow
        val arrowLength = 12.0
        val x2 = robotX + arrowLength * cos(heading)
        val y2 = robotY + arrowLength * sin(heading)

        panelsField.moveCursor(robotX, robotY)
        panelsField.line(x2 = x2, y2 = y2)
    }

    // Kalman state: [x, y, heading] estimates
    private var kX = 0.0
    private var kY = 0.0
    private var kHeading = 0.0

    // Kalman covariances (diagonal of P matrix — one per dimension)
    private var pX = 1.0
    private var pY = 1.0
    private var pH = 1.0

    // Process noise Q: how much uncertainty odometry adds each tick.
    // Increase if the robot drifts noticeably between vision updates.
    private val Q_XY = 100.0      // mm²
    private val Q_H = 0.001      // rad²

    // Measurement noise R: how noisy vision is.
    // Increase if vision jitters; decrease if it's very stable.
    private val R_XY = 500.0      // mm²
    private val R_H = 0.01       // rad²

    override fun periodic(context: BotContext) {
        hardware.pinpoint.update()

        val odometryPose: Pose2D = hardware.pinpoint.position

        var visionPose: Pose2D? = null
        val llResult: LLResult? = hardware.limelight.getLatestResult()
        if (llResult != null && llResult.isValid) {
            val x = llResult.botpose.position.toUnit(DistanceUnit.MM).x
            val y = llResult.botpose.position.toUnit(DistanceUnit.MM).y
            val heading = llResult.botpose.orientation.getYaw(AngleUnit.RADIANS)
            visionPose = Pose2D(DistanceUnit.MM, x, y, AngleUnit.RADIANS, heading)
        }

        when (state) {
            LocalizationState.VISION_ONLY -> {
                if (visionPose != null) pose = visionPose
            }

            LocalizationState.ODOMETRY_ONLY -> {
                pose = odometryPose
            }

            LocalizationState.FUSED -> {
                kalmanFused(odometryPose, visionPose)
            }
        }

        context.botPose = pose

        drawRobot(
            pose.getX(DistanceUnit.INCH),
            pose.getY(DistanceUnit.INCH),
            pose.getHeading(AngleUnit.RADIANS)
        )
        panelsField.update()
    }

    private fun kalmanFused(odometry: Pose2D, vision: Pose2D?) {
        // --- Predict step ---
        // Trust the odometry as our motion model (it's the most continuous source).
        kX = odometry.getX(DistanceUnit.MM)
        kY = odometry.getY(DistanceUnit.MM)
        kHeading = odometry.getHeading(AngleUnit.RADIANS)

        // Grow covariance with process noise each tick.
        pX += Q_XY
        pY += Q_XY
        pH += Q_H

        // --- Update step (only when vision is available) ---
        if (vision != null) {
            // Kalman gain: how much weight to give the vision measurement.
            val kGainX = pX / (pX + R_XY)
            val kGainY = pY / (pY + R_XY)
            val kGainH = pH / (pH + R_H)

            // Innovation: difference between vision measurement and prediction.
            val innovX = vision.getX(DistanceUnit.MM) - kX
            val innovY = vision.getY(DistanceUnit.MM) - kY
            val innovH = wrapAngle(vision.getHeading(AngleUnit.RADIANS) - kHeading)

            // Fuse: pull the estimate toward vision proportional to the gain.
            kX += kGainX * innovX
            kY += kGainY * innovY
            kHeading = wrapAngle(kHeading + kGainH * innovH)

            // Shrink covariance now that we have a measurement.
            pX *= (1.0 - kGainX)
            pY *= (1.0 - kGainY)
            pH *= (1.0 - kGainH)
        }

        pose = Pose2D(DistanceUnit.MM, kX, kY, AngleUnit.RADIANS, kHeading)
    }

    /** Wrap an angle difference into [-π, π] to avoid discontinuities. */
    private fun wrapAngle(angle: Double): Double {
        var a = angle % (2 * PI)
        if (a > PI) a -= 2 * PI
        if (a < -PI) a += 2 * PI
        return a
    }

    fun getPose(): Pose2D = pose
}