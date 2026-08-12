package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

/** Shared radians-based pose factory used by robot code and generated paths. */
public final class Poses {
    private static final PoseFactory RADIANS = PoseFactory.radians();

    private Poses() { }
    public static Pose pose(double x, double y) { return RADIANS.of(x, y, 0); }
    public static Pose pose(double x, double y, double heading) { return RADIANS.of(x, y, heading); }
}
