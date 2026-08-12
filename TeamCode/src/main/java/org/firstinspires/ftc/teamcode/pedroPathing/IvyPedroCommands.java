package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

public final class IvyPedroCommands {
    private IvyPedroCommands() { }

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
                    PathCallbacks.begin(path);
                    follower.follow(path);
                })
                .setExecute(() -> PathCallbacks.update(path, follower))
                .setDone(() -> !follower.isBusy())
                .setEnd(condition -> {
                    follower.setPathSpeed(1.0);
                    if (condition == EndCondition.NATURALLY && holdEnd) follower.hold(path.endPose());
                    else if (condition != EndCondition.NATURALLY) follower.stop();
                })
                .requiring(follower);
    }

    public static Command hold(TeamFollower follower) { return hold(follower, follower.pose()); }
    public static Command hold(TeamFollower follower, Pose pose) {
        return Command.build()
                .setStart(() -> follower.hold(pose))
                .setDone(() -> !follower.isBusy())
                .setEnd(condition -> { if (condition != EndCondition.NATURALLY) follower.stop(); })
                .requiring(follower);
    }

    public static Command turnTo(TeamFollower follower, double heading) {
        return hold(follower, follower.pose().withHeading(heading));
    }
}
