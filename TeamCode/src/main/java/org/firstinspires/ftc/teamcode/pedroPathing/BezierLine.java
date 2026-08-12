package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import static com.pedropathing.api.Paths.line;

public final class BezierLine implements PathSpec {
    private final Pose start;
    private final Pose end;
    public BezierLine(Pose start, Pose end) { this.start = start; this.end = end; }
    @Override public Path toPath() { return line(start, end); }
}
