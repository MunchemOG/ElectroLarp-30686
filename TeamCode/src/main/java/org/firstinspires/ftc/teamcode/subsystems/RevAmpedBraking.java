package org.firstinspires.ftc.teamcode.subsystems;


import com.pedropathing.ivy.Command;
import com.bylazar.configurables.annotations.Configurable;

/**
 * RevAmped's LambertApproximator, ported 1:1.
 * Source: RevAmped-Decode-V2/utils/math/LambertApproximator.java
 *
 * WHAT IT DOES
 * ------------
 * Given the braking-distance fit D(v) = a*v^2 + b*v, the implied deceleration
 * is vdot = -v / (2a*v + b). Separating variables and solving through the
 * Lambert W function gives speed as a function of time into the brake, and the
 * ground covered follows without a second integral because the remaining
 * stopping distance is always D of the current speed.
 *
 * THE PRECOMPUTE TRICK (theirs, kept)
 * -----------------------------------
 * The Lambert W is evaluated ONCE for a fixed horizon and cached as two
 * scalars. At runtime the projection is two multiplies:
 *
 *     distance = distFactor * currentSpeed
 *     speed    = speedFactor * currentSpeed
 *
 * So although the derivation is transcendental, the deployed behaviour is
 * linear in current speed.
 *
 * THE HORIZON IS NOT BALL-EXIT TIME
 * ---------------------------------
 * RevAmped run 0.5s, tuned empirically, and per its author the long lookahead
 * is what gives the lead any effect - which reads as implicit turret-lag
 * compensation rather than a physical prediction of where the ball leaves.
 * That means:
 *
 *   - horizonSeconds is a TUNING KNOB, not feederDelay + turretSettleTime.
 *   - It must NOT be combined with an explicit command-rate feedforward, or
 *     the same servo lag gets corrected twice.
 *
 * UNITS
 * -----
 * Their a = 0.001503 is s^2/METRE. Pedro reports inches, so it is divided by
 * 39.3701 below. Their b = 0.1239 is in seconds and transfers unchanged.
 */
@Configurable
public class RevAmpedBraking {

    /** Quadratic braking coefficient, s^2/in. Theirs, converted from per-metre. */
    public static double coefA = 0.001503 / 39.3701;

    /** Linear braking coefficient, seconds. Theirs verbatim. */
    public static double coefB = 0.1239;

    /** Reference speed their fit was taken at. */
    public static double coefV0 = 72.0;

    /**
     * Projection horizon, seconds. THEIRS IS 0.5 - start here and tune to this
     * robot. Note the cached factors are recomputed whenever this changes, so
     * it is safe to sweep live from Panels.
     */
    public static double horizonSeconds = 0.5;

    private static double cachedSpeedFactor = 0.0;
    private static double cachedDistFactor = 0.0;
    private static double cachedA = Double.NaN;
    private static double cachedB = Double.NaN;
    private static double cachedV0 = Double.NaN;
    private static double cachedHorizon = Double.NaN;
    public static double lambertW(double x) {
        if (x < 0 && x < -1.0 / Math.E) return Double.NaN;

        double w = x > 1.0 ? Math.log(x) - Math.log(Math.log(x)) : x;
        if (w < 0) w = 0.0;

        for (int i = 0; i < 3; i++) {
            double ew = Math.exp(w);
            double f = w * ew - x;
            double num = 2.0 * ew * (1.0 + w);
            double denom = num - (f / (w + 1.0));
            if (Math.abs(denom) < 1e-300) break;
            double delta = f / denom;
            w -= delta;
            if (Math.abs(delta) < 1e-15) break;
        }
        return w;
    }

    /** Their computeX: (2a/b)*v0 * exp((2a/b)*v0 - t/b). */
    private static double computeX(double t) {
        double leadingTerm = (2.0 * coefA / coefB) * coefV0;
        return leadingTerm * Math.exp(leadingTerm - (t / coefB));
    }

    /** Their setDefaultDt: cache the two factors for the current horizon. */
    private static void refreshFactors() {
        double wx0 = lambertW(computeX(0.0));
        double wxNew = lambertW(computeX(horizonSeconds));

        double ratio = (1.0 / coefV0) * coefB / 2.0 / coefA;
        double upper = (wxNew * wxNew / 2.0) + wxNew;
        double lower = (wx0 * wx0 / 2.0) + wx0;
        double integral = coefB * (lower - upper);

        cachedSpeedFactor = wxNew * ratio;
        cachedDistFactor = integral * ratio;

        cachedA = coefA;
        cachedB = coefB;
        cachedV0 = coefV0;
        cachedHorizon = horizonSeconds;
    }

    private static void ensureFresh() {
        if (cachedA != coefA || cachedB != coefB
                || cachedV0 != coefV0 || cachedHorizon != horizonSeconds
                || Double.isNaN(cachedA)) {
            refreshFactors();
        }
    }

    /** Projected speed at the end of the horizon, in/s. */
    public static double speedAfter(double currentSpeed) {
        if (currentSpeed <= 1e-3) return 0.0;
        ensureFresh();
        double v = cachedSpeedFactor * currentSpeed;
        if (Double.isNaN(v) || v < 0.0) return 0.0;
        return Math.min(v, currentSpeed);   // braking cannot speed you up
    }

    /** Ground covered during the horizon, inches. */
    public static double distanceAfter(double currentSpeed) {
        if (currentSpeed <= 1e-3) return 0.0;
        ensureFresh();
        double d = cachedDistFactor * currentSpeed;
        if (Double.isNaN(d) || d < 0.0) return 0.0;
        return d;
    }

    /** Deceleration this model implies at a given speed, in/s^2. Diagnostic:
     *  compare against the measured aAlongV telemetry to see how well their
     *  fit describes this drivetrain. */
    public static double impliedDecel(double currentSpeed) {
        if (currentSpeed <= 1e-3 || horizonSeconds <= 1e-6) return 0.0;
        return (speedAfter(currentSpeed) - currentSpeed) / horizonSeconds;
    }
}