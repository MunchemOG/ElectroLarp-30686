package org.firstinspires.ftc.teamcode.opModes.Auto;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.pedroPathing.TeamFollower;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.WeakHashMap;

/** Ivy integration for Pedro 3 paths, scoped to autonomous OpModes. */
public final class AutoPathRuntime {
    private static final Map<Path, List<Callback>> CALLBACKS = new WeakHashMap<>();
    private static long startedNanos;

    private AutoPathRuntime() { }

    public static Callback temporalCallback(double time, Command command) {
        return new Callback(time > 10 ? time / 1000.0 : time, null, 0, command);
    }

    public static Callback poseCallback(Pose pose, Command command, double tolerance) {
        return new Callback(-1, pose, tolerance, command);
    }

    public static Path withCallbacks(Path path, Callback... callbacks) {
        List<Callback> definitions = new ArrayList<>();
        Collections.addAll(definitions, callbacks);
        CALLBACKS.put(path, definitions);
        return path;
    }

    public static Command follow(TeamFollower follower, Path path) {
        return follow(follower, path, false, 1.0);
    }

    public static Command follow(TeamFollower follower, Path path, boolean holdEnd) {
        return follow(follower, path, holdEnd, 1.0);
    }

    public static Command follow(TeamFollower follower, Path path, double maxPower) {
        return follow(follower, path, false, maxPower);
    }

    public static Command follow(TeamFollower follower, Path path, boolean holdEnd, double maxPower) {
        return Command.build()
                .setStart(() -> {
                    follower.setPathSpeed(maxPower);
                    startedNanos = System.nanoTime();
                    for (Callback callback : CALLBACKS.getOrDefault(path, Collections.emptyList())) {
                        callback.fired = false;
                    }
                    follower.follow(path);
                })
                .setExecute(() -> updateCallbacks(path, follower))
                .setDone(() -> !follower.isBusy())
                .setEnd(condition -> {
                    follower.setPathSpeed(1.0);
                    if (condition == EndCondition.NATURALLY && holdEnd) follower.hold(path.endPose());
                    else if (condition != EndCondition.NATURALLY) follower.stop();
                })
                .requiring(follower);
    }

    public static Command hold(TeamFollower follower) {
        return hold(follower, follower.pose());
    }

    public static Command hold(TeamFollower follower, Pose pose) {
        return Command.build()
                .setStart(() -> follower.hold(pose))
                .setDone(() -> !follower.isBusy())
                .setEnd(condition -> {
                    if (condition != EndCondition.NATURALLY) follower.stop();
                })
                .requiring(follower);
    }

    public static Command turnTo(TeamFollower follower, double heading) {
        return hold(follower, follower.pose().withHeading(heading));
    }

    private static void updateCallbacks(Path path, TeamFollower follower) {
        double elapsed = (System.nanoTime() - startedNanos) / 1e9;
        for (Callback callback : CALLBACKS.getOrDefault(path, Collections.emptyList())) {
            if (callback.fired) continue;
            boolean ready = callback.pose == null
                    ? elapsed >= callback.seconds
                    : follower.pose().distance(callback.pose) <= callback.tolerance;
            if (ready) {
                callback.fired = true;
                callback.command.schedule();
            }
        }
    }

    public static final class Callback {
        private final double seconds;
        private final Pose pose;
        private final double tolerance;
        private final Command command;
        private boolean fired;

        private Callback(double seconds, Pose pose, double tolerance, Command command) {
            this.seconds = seconds;
            this.pose = pose;
            this.tolerance = tolerance;
            this.command = command;
        }
    }
}
