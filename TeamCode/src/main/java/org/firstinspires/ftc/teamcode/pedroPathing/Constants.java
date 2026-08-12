package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.math.Matrix;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.Pinpoint;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/** Pedro Pathing 3 robot configuration translated from the team's 2.1 constants. */
public final class Constants {
    private Constants() { }

    public static final MecanumConfig driveConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("frontLeft");
        c.backLeftName.set("backLeft");
        c.frontRightName.set("frontRight");
        c.backRightName.set("backRight");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.manualBrakeMode.set(true);
    });

    public static final PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.xPodOffset.set(4.955);
        c.yPodOffset.set(0.89);
        c.offsetUnits.set(DistanceUnit.INCH);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
    });

    public static final ForesightConfig foresightConfig = new ForesightConfig(c -> {
        c.robotMass.set(10.0);
        c.brakeController.set(Controller.pid(0.18, 0, 0));
        c.headingController.set(Controller.pid(1.77, 0, 0.2235));
        c.linearBrakeCoefficients.set(Matrix.diag(0.11091, 0.11091));
        c.quadraticBrakeCoefficients.set(Matrix.diag(0.00097587, 0.00097587));
        c.naturalForwardDeceleration.set(52.0);
        c.naturalStrafeDeceleration.set(83.0);
        c.centripetalScaling.set(0.0);
        c.parametricTConstraint.set(0.9);
        c.velocityConstraint.set(0.8);
        c.translationalConstraint.set(0.8);
        c.headingConstraint.set(0.5);
        c.timeoutConstraint.set(50.0);
    });

    public static TeamFollower create(HardwareMap hardwareMap) {
        return new TeamFollower(
                new Pinpoint(hardwareMap, localizerConfig),
                new Mecanum(hardwareMap, driveConfig),
                new Foresight(foresightConfig),
                foresightConfig);
    }

    public static TeamFollower createFollower(HardwareMap hardwareMap) {
        return create(hardwareMap);
    }
}
