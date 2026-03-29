package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.hardware.limelightvision.LLResult
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.teamcode.enums.LocalizationState
import org.firstinspires.ftc.teamcode.hardware.RobotHardware
import org.firstinspires.ftc.teamcode.interfaces.Subsystem
import kotlin.math.PI

class Localization(private val hardware: RobotHardware) : Subsystem {
    private var state: LocalizationState = LocalizationState.FUSED

    private var pose: Pose2D = Pose2D(DistanceUnit.MM, 0.0, 0.0, AngleUnit.RADIANS, 0.0)

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
    private val Q_H  = 0.001      // rad²

    // Measurement noise R: how noisy vision is.
    // Increase if vision jitters; decrease if it's very stable.
    private val R_XY = 500.0      // mm²
    private val R_H  = 0.01       // rad²

    override fun periodic() {
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
    }

    private fun kalmanFused(odometry: Pose2D, vision: Pose2D?) {
        // --- Predict step ---
        // Trust the odometry as our motion model (it's the most continuous source).
        kX       = odometry.getX(DistanceUnit.MM)
        kY       = odometry.getY(DistanceUnit.MM)
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
            val innovX = vision.getX(DistanceUnit.MM)       - kX
            val innovY = vision.getY(DistanceUnit.MM)       - kY
            val innovH = wrapAngle(vision.getHeading(AngleUnit.RADIANS) - kHeading)

            // Fuse: pull the estimate toward vision proportional to the gain.
            kX       += kGainX * innovX
            kY       += kGainY * innovY
            kHeading  = wrapAngle(kHeading + kGainH * innovH)

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
        if (a > PI)  a -= 2 * PI
        if (a < -PI) a += 2 * PI
        return a
    }

    fun getPose(): Pose2D = pose
}