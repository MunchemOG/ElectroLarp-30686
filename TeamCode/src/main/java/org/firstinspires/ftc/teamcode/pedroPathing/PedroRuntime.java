package org.firstinspires.ftc.teamcode.pedroPathing;

public final class PedroRuntime {
    private static TeamFollower follower;
    private PedroRuntime() { }
    public static void attach(TeamFollower activeFollower) { follower = activeFollower; }
    public static void detach() { follower = null; }
    public static TeamFollower follower() {
        if (follower == null) throw new IllegalStateException("Pedro follower is not initialized");
        return follower;
    }
}
