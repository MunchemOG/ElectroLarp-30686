package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import java.util.ArrayList;
import java.util.List;

import static com.pedropathing.api.Paths.path;

/** Pedro 2 path-builder surface translated onto Pedro 3 immutable Path factories. */
public final class PathBuilder {
    private final ForesightConfig config;
    private final List<Path> paths = new ArrayList<>();
    private final List<PathCallbacks.Callback> callbacks = new ArrayList<>();

    PathBuilder(ForesightConfig config) { this.config = config; }

    public PathBuilder addPath(PathSpec spec) { paths.add(spec.toPath()); return this; }

    private Path last() {
        if (paths.isEmpty()) throw new IllegalStateException("Add a path before configuring it");
        return paths.get(paths.size() - 1);
    }

    private PathBuilder replaceLast(Path replacement) {
        paths.set(paths.size() - 1, replacement);
        return this;
    }

    public PathBuilder setTangentHeadingInterpolation() { return replaceLast(last().tangent()); }
    public PathBuilder setLinearHeadingInterpolation(double start, double end) { return replaceLast(last().linear(start, end)); }
    /** Pedro 3 interpolates over the complete path; retained for migrated Pedro 2 call sites. */
    public PathBuilder setLinearHeadingInterpolation(double start, double end, double ignoredEndT) {
        return setLinearHeadingInterpolation(start, end);
    }
    public PathBuilder setConstantHeadingInterpolation(double heading) { return replaceLast(last().constant(heading)); }
    public PathBuilder setReversed() { return replaceLast(last().reverseTangent()); }
    public PathBuilder setTValueConstraint(double value) { return replaceLast(last().with(config.parametricTConstraint.at(value))); }
    public PathBuilder setVelocityConstraint(double value) { return replaceLast(last().with(config.velocityConstraint.at(value))); }
    public PathBuilder setBrakingStrength(double value) { return replaceLast(last().with(config.brakeAggression.at(value))); }

    public PathBuilder addTemporalCallback(double time, Command command) {
        callbacks.add(new PathCallbacks.Callback(time, command));
        return this;
    }

    public PathBuilder addPoseCallback(Pose pose, Command command, double tolerance) {
        callbacks.add(new PathCallbacks.Callback(pose, command, tolerance));
        return this;
    }

    public Path build() {
        if (paths.isEmpty()) throw new IllegalStateException("Cannot build an empty path");
        Path result = path(paths.toArray(new Path[0]));
        PathCallbacks.register(result, callbacks);
        return result;
    }
}
