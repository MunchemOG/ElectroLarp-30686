package org.firstinspires.ftc.teamcode.pedroPathing;

public final class MathFunctions {
    private MathFunctions() { }

    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public static double normalizeAngle(double angle) {
        double normalized = angle % (2 * Math.PI);
        return normalized < 0 ? normalized + 2 * Math.PI : normalized;
    }
}
