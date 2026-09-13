package org.firstinspires.ftc.teamcode.opModes.Auto;

import com.pedropathing.api.PoseFactory;
import static com.pedropathing.api.Paths.*;


import org.firstinspires.ftc.teamcode.ivy.*;
import org.firstinspires.ftc.teamcode.pedroPathing.*;

import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.*;
import static org.firstinspires.ftc.teamcode.ivy.HardwareCommands.*;
import static org.firstinspires.ftc.teamcode.opModes.Auto.AutoPathRuntime.*;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.MathFunctions;

import java.util.Arrays;

public class ColoredDecodePose {

    private static final PoseFactory POSES = PoseFactory.radians();
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
        this(POSES.of(posX, posY, heading), allianceColor);
    }

    public ColoredDecodePose(Pose pose, AllianceColor color) {
        if (color == AllianceColor.None) throw new RuntimeException("Uncolored ColoredDecodePose");
        this.pose = pose;
        this.color = color;
        if (color == AllianceColor.Red) red = pose;
        else if (color == AllianceColor.Blue) blue = pose;
    }

    public ColoredDecodePose(double x, double y, double heading) {
        this(POSES.of(x, y, heading), AllianceColor.Blue);
    }

    public ColoredDecodePose(double x, double y) {
        this(POSES.of(x, y, 0), AllianceColor.Blue);
    }

    public ColoredDecodePose() {
        this(0, 0);
    }

    public Pose getPose(AllianceColor desiredColor) {
        if (desiredColor.equals(AllianceColor.Red)) {
            if (red == null) red = mirror(pose);
            return red;
        }

        if (blue == null) blue = mirror(pose);
        return blue;
    }

    public Pose getPose() {
        return getPose(Globals.allianceColor);
    }

    public ColoredDecodePose down(double inches) {
        return new ColoredDecodePose(this.pose.plus(POSES.of(0, -inches, 0)), color);
    }

    public ColoredDecodePose up(double inches) {
        return new ColoredDecodePose(this.pose.plus(POSES.of(0, inches, 0)), color);
    }

    public ColoredDecodePose towardsRedWall(double inches) {
        return new ColoredDecodePose(this.pose.plus(POSES.of(color == AllianceColor.Blue ? inches : -inches, 0, 0)), color);
    }

    public ColoredDecodePose towardsBlueWall(double inches) {
        return new ColoredDecodePose(this.pose.plus(POSES.of(color == AllianceColor.Red ? inches : -inches, 0, 0)), color);
    }

    public static Path makeBezier(ColoredDecodePose... poses) {
        return curve(Arrays.stream(poses).map(ColoredDecodePose::getPose).toArray(Pose[]::new));
    }

    public static Path through(ColoredDecodePose... poses) {
        return com.pedropathing.api.Paths.through(
                Arrays.stream(poses).map(ColoredDecodePose::getPose).toArray(Pose[]::new));
    }

    public static Path makeBezier(ColoredDecodePose pose1, ColoredDecodePose pose2) {
        return line(pose1.getPose(), pose2.getPose());
    }

    private static Pose mirror(Pose source) {
        return POSES.of(144 - source.x(), source.y(),
                MathFunctions.normalizeAngle(Math.PI - source.heading()));
    }

    public AllianceColor getColor() {
        return color;
    }

    public Pose getUnmodifiedPose() {
        return pose;
    }

    public double getHeading() {
        return getPose().heading();
    }

    public ColoredDecodePose offsetOppositeColor(Pose offset) {
        if (blue == null || red == null) getPose();

        if (color == AllianceColor.Red) blue = blue.plus(offset);
        if (color == AllianceColor.Blue) red = red.plus(offset);
        return this;
    }

    public static double getTangentHeading(ColoredDecodePose pose1, ColoredDecodePose pose2) {
        return Math.atan2(pose2.getPose().y() - pose1.getPose().y(), pose2.getPose().x() - pose1.getPose().x());
    }

    public static double getHeading(double heading) {
        if (Globals.allianceColor.equals(AllianceColor.Red)) return Math.PI - heading;
        return heading;
    }

}
