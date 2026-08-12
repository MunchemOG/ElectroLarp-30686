package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.WeakHashMap;

final class PathCallbacks {
    static final class Callback {
        final double seconds;
        final Pose pose;
        final double tolerance;
        final Command command;
        boolean fired;

        Callback(double seconds, Command command) {
            this.seconds = seconds > 10 ? seconds / 1000.0 : seconds;
            this.pose = null;
            this.tolerance = 0;
            this.command = command;
        }

        Callback(Pose pose, Command command, double tolerance) {
            this.seconds = -1;
            this.pose = pose;
            this.tolerance = tolerance;
            this.command = command;
        }
    }

    private static final Map<Path, List<Callback>> callbacks = new WeakHashMap<>();
    private static long startedNanos;

    private PathCallbacks() { }

    static void register(Path path, List<Callback> definitions) {
        callbacks.put(path, new ArrayList<>(definitions));
    }

    static void begin(Path path) {
        startedNanos = System.nanoTime();
        for (Callback callback : callbacks.getOrDefault(path, Collections.emptyList())) callback.fired = false;
    }

    static void update(Path path, TeamFollower follower) {
        double elapsed = (System.nanoTime() - startedNanos) / 1e9;
        for (Callback callback : callbacks.getOrDefault(path, Collections.emptyList())) {
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
}
