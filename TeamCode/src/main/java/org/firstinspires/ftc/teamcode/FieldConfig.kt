package org.firstinspires.ftc.teamcode

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.teamcode.enums.Team

object FieldConfig {
    val redGoalPose = Pose2D(DistanceUnit.INCH, -72.0, 72.0, AngleUnit.RADIANS, 0.0)
    val redShootingPose = Pose2D(DistanceUnit.INCH, -24.0, 24.0, AngleUnit.RADIANS, Math.toRadians(135.0))
    val redIntakingPose = Pose2D(DistanceUnit.INCH, -24.0, -10.0, AngleUnit.RADIANS, Math.toRadians(130.0))

    fun mirror(pose: Pose2D, team: Team): Pose2D {
        if (team == Team.BLUE) {
            val mirroredX = -pose.getX(DistanceUnit.INCH)
            val mirroredHeading = Math.PI - pose.getHeading(AngleUnit.RADIANS)
            return Pose2D(DistanceUnit.INCH, mirroredX, pose.getY(DistanceUnit.INCH), AngleUnit.RADIANS, mirroredHeading)
        }
        return pose
    }
}