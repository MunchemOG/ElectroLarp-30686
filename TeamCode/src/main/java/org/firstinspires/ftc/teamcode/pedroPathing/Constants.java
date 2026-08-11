package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10)
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.18, 0.11091,0.00097587))
            .forwardZeroPowerAcceleration(-52)
            .lateralZeroPowerAcceleration(-83)
            .headingPIDFCoefficients(new PIDFCoefficients(
                    1.77,
                    0,
                    0.2235,
                    0
            ))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(
                    0.7895,
                    0,
                    0.1171,
                    0
            ))
            .centripetalScaling(0.000);
    public static MecanumConstants driveConstants = new MecanumConstants()
            .useBrakeModeInTeleOp(true)
            .leftFrontMotorName("frontLeft")
            .leftRearMotorName("backLeft")
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE);
    //.xVelocity(97)
    //.yVelocity(80);

    public static PinpointConstants localizerConstants = new PinpointConstants()
//            .forwardPodY(4.86)
//            .strafePodX(0.84)
            .forwardPodY(4.955)
            .strafePodX(0.89)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.9,
            0.8,
            0.8,
            0.5,
            50,
            3.5,
            10,
            1

    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}