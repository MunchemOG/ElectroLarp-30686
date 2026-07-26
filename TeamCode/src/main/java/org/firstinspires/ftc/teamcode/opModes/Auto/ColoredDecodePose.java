package org.firstinspires.ftc.teamcode.opModes.Auto;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.FuturePose;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.HeadingInterpolator;

import java.util.Arrays;

public class ColoredDecodePose implements FuturePose {
    private final Pose pose;
    private final AllianceColor color;
    private Pose blue;
    private Pose red;

    public ColoredDecodePose(Pose blue, Pose red) {
        pose = blue;
        color = AllianceColor.Blue;
        this.blue = blue;
        this.red = red;
    }

    public ColoredDecodePose(double posX, double posY, double heading, AllianceColor allianceColor) {
        this(new Pose(posX, posY, heading), allianceColor);
    }

    public ColoredDecodePose(Pose pose, AllianceColor color) {
        if (color == AllianceColor.None) throw new RuntimeException("Uncolored ColoredDecodePose");
        this.pose = pose;
        this.color = color;
        if (color == AllianceColor.Red) red = pose;
        else if (color == AllianceColor.Blue) blue = pose;
    }

    public ColoredDecodePose(double x, double y, double heading) {
        this(new Pose(x, y, heading), AllianceColor.Blue);
    }

    public ColoredDecodePose(double x, double y) {
        this(new Pose(x, y), AllianceColor.Blue);
    }

    public ColoredDecodePose() {
        this(0, 0);
    }

    public Pose getPose(AllianceColor desiredColor) {
        if (desiredColor.equals(AllianceColor.Red)) {
            if (red == null) red = pose.mirror();
            return red;
        }

        if (blue == null) blue = pose.mirror();
        return blue;
    }

    @Override
    public Pose getPose() {
        return getPose(Globals.allianceColor);
    }

    public ColoredDecodePose down(double inches) {
        return new ColoredDecodePose(this.pose.plus(new Pose(0, -inches)), color);
    }

    public ColoredDecodePose up(double inches) {
        return new ColoredDecodePose(this.pose.plus(new Pose(0, inches)), color);
    }

    public ColoredDecodePose towardsRedWall(double inches) {
        return new ColoredDecodePose(this.pose.plus(new Pose(color == AllianceColor.Blue ? inches : -inches, 0)), color);
    }

    public ColoredDecodePose towardsBlueWall(double inches) {
        return new ColoredDecodePose(this.pose.plus(new Pose(color == AllianceColor.Red ? inches : -inches, 0)), color);
    }

    public static BezierCurve makeBezier(ColoredDecodePose... poses) {
        return new BezierCurve(Arrays.stream(poses).map(ColoredDecodePose::getPose).toArray(Pose[]::new));
    }

    public static BezierCurve through(ColoredDecodePose... poses) {
        return BezierCurve.through(Arrays.stream(poses).map(ColoredDecodePose::getPose).toArray(Pose[]::new));
    }

    public static BezierLine makeBezier(ColoredDecodePose pose1, ColoredDecodePose pose2) {
        return new BezierLine(pose1.getPose(), pose2.getPose());
    }

    public static BezierPoint makeBezier(ColoredDecodePose pose) {
        return new BezierPoint(pose.getPose());
    }

    public static HeadingInterpolator mirror(HeadingInterpolator interpolation) {
        return t -> MathFunctions.normalizeAngle(Math.PI - interpolation.interpolate(t));
    }

    public AllianceColor getColor() {
        return color;
    }

    public Pose getUnmodifiedPose() {
        return pose;
    }

    public double getHeading() {
        return getPose().getHeading();
    }

    public ColoredDecodePose offsetOppositeColor(Pose offset) {
        if (blue == null || red == null) getPose();

        if (color == AllianceColor.Red) blue = blue.plus(offset);
        if (color == AllianceColor.Blue) red = red.plus(offset);
        return this;
    }

    public static double getTangentHeading(ColoredDecodePose pose1, ColoredDecodePose pose2) {
        return Math.atan2(pose2.getPose().getY() - pose1.getPose().getY(), pose2.getPose().getX() - pose1.getPose().getX());
    }

    public static double getHeading(double heading) {
        if (Globals.allianceColor.equals(AllianceColor.Red)) return Math.PI - heading;
        return heading;
    }

    @Override
    public boolean initialized() {
        return true;
    }
}
