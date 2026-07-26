package org.firstinspires.ftc.teamcode.utils.control;

import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.control.PredictiveBrakingController;

public class SquIDBrakingController extends PredictiveBrakingController {
    private final PredictiveBrakingCoefficients coefficients;

    public SquIDBrakingController(PredictiveBrakingCoefficients coefficients) {
        super(coefficients);
        this.coefficients = coefficients;
    }

    @Override
    public double computeOutput(double error, double velocity) {
        double directionOfMotion = Math.signum(velocity);
        double realError = error - computeBrakingDisplacement(velocity, directionOfMotion);
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
