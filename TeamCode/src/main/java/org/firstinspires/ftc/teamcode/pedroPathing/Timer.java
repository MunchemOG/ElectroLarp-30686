package org.firstinspires.ftc.teamcode.pedroPathing;

/** Compatibility facade over Pedro 3's timer names. */
public final class Timer {
    private final com.pedropathing.utils.Timer timer = new com.pedropathing.utils.Timer();
    public void resetTimer() { timer.reset(); }
    public double getElapsedTimeSeconds() { return timer.seconds(); }
    public double getElapsedTime() { return timer.milliseconds(); }
}
