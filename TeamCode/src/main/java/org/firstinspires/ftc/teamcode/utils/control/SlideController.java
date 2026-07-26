package org.firstinspires.ftc.teamcode.utils.control;

import com.qualcomm.robotcore.util.Range;

public class SlideController {
    //Constants
    public double kGravity; //control-input required for steady-state to counteract forces like gravity, unneeded for hslides
    public double kD; //velocity damping
    public double kP; //proportional
    public double brakeBias; //lower to stop overshoot
    public double quadraticBrake; //physics-based brake contribution
    public double linearBrake; //drag
    public double constantBrake; //gravity contribution to brake, unneeded for hslides
    private double maxPower = 1.0;
    private double minPower = -0.2;
    private static final double epsilon = 1e-3;

    private SlideController(double kGravity, double kD, double kP, double brakeBias, double quadraticBrake, double linearBrake, double constantBrake) {
        this.kGravity = kGravity;
        this.kD = kD;
        this.kP = kP;
        this.brakeBias = brakeBias;
        this.quadraticBrake = quadraticBrake;
        this.linearBrake = linearBrake;
        this.constantBrake = constantBrake;
    }

    public static SlideController hslideController(double kV, double kP, double quadraticBrake, double linearBrake, double brakeBias) {
        return new SlideController(0, kV, kP, brakeBias, quadraticBrake, linearBrake, 0);
    }

    public static SlideController hslideController(double kV, double kP, double quadraticBrake, double linearBrake) {
        return hslideController(kV, kP, quadraticBrake, linearBrake, 1.0);
    }

    public static SlideController vslideController(double kGravity, double kV, double kP, double quadraticBrake, double linearBrake, double constantBrake, double brakeBias) {
        return new SlideController(kGravity, kV, kP, brakeBias, quadraticBrake, linearBrake, constantBrake);
    }

    public static SlideController vslideController(double kGravity, double kV, double kP, double quadraticBrake, double linearBrake, double constantBrake) {
        return vslideController(kGravity, kV, kP, quadraticBrake, linearBrake, constantBrake, 1.0);
    }

    public void setMaxPower(double maxPower) {
        this.maxPower = maxPower;
    }

    public void setMinPower(double minPower) {
        this.minPower = minPower;
    }

    public double brakingDistance(double velocity) {
        velocity = Math.abs(velocity);
        return constantBrake + linearBrake * velocity + quadraticBrake * velocity * velocity;
    }

    private double applyController(double error, double velocity) {
        return kP * error - kD * velocity + kGravity;
    }

    public double calculate(double error, double velocity) {
        if (error * velocity < 0) return applyController(error, velocity);
        double errorDir = Math.signum(error);
        double unsignedError = Math.abs(error);
        double effectiveError = unsignedError - brakeBias * brakingDistance(velocity);
        if (effectiveError > epsilon) return maxPower * errorDir;
        return Range.clip(applyController(effectiveError * errorDir, velocity), minPower, maxPower);
    }
}
