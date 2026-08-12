package org.firstinspires.ftc.teamcode.utils.control;

/** Project-specific square-root braking controller retained independently of Pedro internals. */
public class SquIDBrakingController {
    public static final class Coefficients {
        public final double P;
        public final double maximumBrakingPower;

        public Coefficients(double p, double maximumBrakingPower) {
            this.P = p;
            this.maximumBrakingPower = maximumBrakingPower;
        }
    }

    private final Coefficients coefficients;

    public SquIDBrakingController(Coefficients coefficients) {
        this.coefficients = coefficients;
    }

    public double computeOutput(double error, double velocity) {
        double directionOfMotion = Math.signum(velocity);
        double realError = error;
        double outputPower = coefficients.P * Math.signum(realError) * Math.sqrt(Math.abs(realError));
        return clampReversePower(outputPower, directionOfMotion);
    }

    /**
     * Prevents the controller from applying too much power in the opposite direction of
     * the robot's momentum. Alternating full forward (+1) and full reverse (-1) power
     * caused the control hub to restart due to low voltage spikes. This fixes it by
     * capping the amount of voltage applied opposite to the direction of motion to be
     * very minimal. Even a tiny opposite voltage (e.g., -0.0001) locks the wheels like
     * zero-power brake mode, using the motor’s own momentum for braking without consuming
     * significant energy.
     */
    private double clampReversePower(double power, double directionOfMotion) {
        boolean isOpposingMotion = directionOfMotion * power < 0;
        if (!isOpposingMotion) {
            return power;
        }
        double clampedPower;
        if (power < 0) {
            clampedPower = Math.max(power, -coefficients.maximumBrakingPower);
        } else {
            clampedPower = Math.min(power, coefficients.maximumBrakingPower);
        }
        return clampedPower;
    }
}
