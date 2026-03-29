package org.firstinspires.ftc.teamcode

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D

object FieldConfig {
    val redGoalPose = Pose2D(DistanceUnit.INCH, -72.0, 72.0, AngleUnit.RADIANS, 0.0)
    val redShootingPose = Pose2D(DistanceUnit.INCH, -24.0, 24.0, AngleUnit.RADIANS, Math.toRadians(135.0))
    val redIntakingPose = Pose2D(DistanceUnit.INCH, -24.0, -10.0, AngleUnit.RADIANS, Math.toRadians(130.0))
}