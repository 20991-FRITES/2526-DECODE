package org.firstinspires.ftc.teamcode.config

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain

object AutoPaths {
    val startPose = Pose(122.0, 122.0, Math.toRadians(45.0))
    val shootPose = Pose(84.0, 84.0, Math.toRadians(45.0))
    val intakePose = Pose(126.0, 61.0, Math.toRadians(35.0))
    val northRowEntryPose = Pose(99.0, 84.0, Math.toRadians(0.0))
    val middleRowEntryPose = Pose(99.0, 60.0, Math.toRadians(0.0))
    val parkPose = Pose(84.0, 105.0, Math.toRadians(0.0))

    fun buildMoveToShootPath(start: Pose, follower: Follower): PathChain {
        return follower.pathBuilder()
            .addPath(BezierLine(start, shootPose))
            .setLinearHeadingInterpolation(start.heading, shootPose.heading)
            .build()
    }

    fun buildIntakeFromRampPath(lastPathEndPose: Pose, follower: Follower): PathChain {
        return follower.pathBuilder()
            .addPath(BezierLine(lastPathEndPose, intakePose))
            .setLinearHeadingInterpolation(lastPathEndPose.heading, intakePose.heading)
            .build()
    }

    fun buildMoveToNorthRowPath(lastPathEndPose: Pose, follower: Follower): PathChain {
        return follower.pathBuilder()
            .addPath(BezierLine(lastPathEndPose, northRowEntryPose))
            .setLinearHeadingInterpolation(lastPathEndPose.heading, northRowEntryPose.heading)
            .build()
    }

    fun buildMoveToMiddleRowPath(lastPathEndPose: Pose, follower: Follower): PathChain {
        return follower.pathBuilder()
            .addPath(BezierLine(lastPathEndPose, middleRowEntryPose))
            .setLinearHeadingInterpolation(lastPathEndPose.heading, middleRowEntryPose.heading)
            .build()
    }

    fun buildIntakeStraightPath(lastPathEndPose: Pose, follower: Follower): PathChain {
        val intakeStraightPose = Pose(intakePose.x + 30.0, intakePose.y, 0.0)
        return follower.pathBuilder()
            .addPath(BezierLine(lastPathEndPose, intakeStraightPose))
            .setLinearHeadingInterpolation(lastPathEndPose.heading, intakeStraightPose.heading)
            .build()
    }

    fun buildMoveToParkPath(lastPathEndPose: Pose, follower: Follower): PathChain {
        return follower.pathBuilder()
            .addPath(BezierLine(lastPathEndPose, parkPose))
            .setLinearHeadingInterpolation(lastPathEndPose.heading, parkPose.heading)
            .build()
    }
}