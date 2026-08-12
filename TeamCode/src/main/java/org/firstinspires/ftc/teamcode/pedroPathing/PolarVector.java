package org.firstinspires.ftc.teamcode.pedroPathing;

/** Polar vector used by the shooter model; separate from Pedro 3's N-dimensional Vector. */
public final class PolarVector {
    private double magnitude;
    private final double theta;

    public PolarVector() { this(0, 0); }
    public PolarVector(double magnitude, double theta) {
        this.magnitude = magnitude;
        this.theta = theta;
    }

    public static PolarVector cartesian(double x, double y) {
        return new PolarVector(Math.hypot(x, y), Math.atan2(y, x));
    }

    public double getMagnitude() { return magnitude; }
    public double getTheta() { return theta; }
    public double getXComponent() { return magnitude * Math.cos(theta); }
    public double getYComponent() { return magnitude * Math.sin(theta); }
    public void setMagnitude(double magnitude) { this.magnitude = magnitude; }
    public PolarVector times(double scalar) {
        return cartesian(getXComponent() * scalar, getYComponent() * scalar);
    }
}
