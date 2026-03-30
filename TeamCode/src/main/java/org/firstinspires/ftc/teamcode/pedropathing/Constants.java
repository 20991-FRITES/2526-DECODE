package org.firstinspires.ftc.teamcode.pedropathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import config.HardwareConfig;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.enums.Team;

public class Constants {
    public static FollowerConstants followerConstants =
            new FollowerConstants()
                    .mass(15) // TODO: Set this
                    .forwardZeroPowerAcceleration(-29.75878312266954)
                    .lateralZeroPowerAcceleration(-63.18270607279411)
                    .translationalPIDFCoefficients(new PIDFCoefficients(0.075, 0, 0.001, 0.025))
                    .headingPIDFCoefficients(new PIDFCoefficients(0.85, 0, 0.001, 0.025))
                    .drivePIDFCoefficients(
                            new FilteredPIDFCoefficients(0.05, 0, 0.00001, 0.6, 0.025));

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static MecanumConstants driveConstants =
            new MecanumConstants()
                    .maxPower(1)
                    .rightFrontMotorName("front_right")
                    .rightRearMotorName("back_right")
                    .leftRearMotorName("back_left")
                    .leftFrontMotorName("front_left")
                    .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
                    .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
                    .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
                    .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
                    .xVelocity(64.05772772360976)
                    .yVelocity(49.779096948818896);

    public static PinpointConstants localizerConstants =
            new PinpointConstants()
                    .forwardPodY(HardwareConfig.PINPOINT_OFFSET_X.getValue(DistanceUnit.INCH))
                    .strafePodX(HardwareConfig.PINPOINT_OFFSET_Y.getValue(DistanceUnit.INCH))
                    .distanceUnit(DistanceUnit.INCH)
                    .hardwareMapName("pinpoint")
                    .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
                    .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
                    .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
