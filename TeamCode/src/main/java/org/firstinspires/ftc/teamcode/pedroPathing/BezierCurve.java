package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import static com.pedropathing.api.Paths.curve;

public final class BezierCurve implements PathSpec {
    private final Pose[] poses;
    public BezierCurve(Pose... poses) { this.poses = poses; }
    @Override public Path toPath() { return curve(poses); }
}
