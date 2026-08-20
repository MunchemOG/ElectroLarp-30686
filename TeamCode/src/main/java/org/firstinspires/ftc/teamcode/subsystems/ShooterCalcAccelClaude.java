package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.ShooterConstants.SCORE_ANGLE;
import static java.lang.Double.isNaN;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

/**
 * Shoot-on-the-move solver.
 *
 * Restructured to follow "On-Bot Control & Dynamic Targeting" (Control Lab,
 * Module 7, Advanced Research):
 *
 *   Step 3 - SOTM is a fixed-point problem. The virtual target changes the
 *            distance, the distance changes flight time, and flight time
 *            changes the virtual target. Iterate to a tolerance with a hard
 *            iteration cap so the real-time loop always returns.
 *
 *   Step 4 - The robot is braking while the shot happens. Project the pose
 *            and velocity forward with the closed-form coast model and hand
 *            the PROJECTED state to the solver. For a servo turret this
 *            projection doubles as the velocity reference (Step 5).
 *
 * The lesson writes the lead as G*v*(t_d + t_f). This implementation splits
 * that product: t_d (feeder delay + turret settle) is applied by projecting the
 * robot's launch state forward through the coast, and t_f is applied as the
 * lead inside the fixed-point loop. That split is deliberate and matters,
 * because the robot's speed is not constant across the two intervals - it is
 * still decaying. Using one averaged velocity across (t_d + t_f) leads wrong.
 */
@Configurable
public class ShooterCalcAccelClaude implements Subsystem {

    // ---- Step 3: fixed-point solver parameters -----------------------------

    /** Feeder / launch delay before free flight, seconds (t_d in the lesson).
     *  Set to 0.2 based on the team's own estimate, matching the lookahead
     *  already used by the auto's predictive launch-zone check.
     *  Worth confirming on high-frame-rate video: stopper-open to ball-exit. */
    public static double feederDelay = 0.1;

    /** Servo turret transport lag, seconds. Added to the projection horizon so
     *  the turret is commanded toward where the shot will be taken rather than
     *  chasing. The lesson's Step 5 note: a servo turret has a position-only
     *  interface, so aiming at the projected pose is the velocity reference. */
    public static double turretSettleTime = 0.03;

    /** Empirical lead gain (G in the lesson). 1.0 = pure geometric lead. */
    public static double leadGain = 1;

    /** Convergence tolerance on flight time, seconds. */
    public static double solveTolerance = 0.001;

    /** Hard iteration cap. The lesson notes convergence normally takes 2-3
     *  rounds; the cap exists only to bound the loop if the model misbehaves. */
    public static int maxIterations = 5;

    // ---- Step 4: coast projection -----------------------------------------

    // ---- Step 4: launch-state projection -----------------------------------

    /** Project using measured along-track acceleration. Default: adapts to
     *  coasting, active braking, and powered acceleration alike. */
    public static final int MODE_MEASURED_ACCEL = 0;
    /** No projection: assume constant velocity through the horizon. */
    public static final int MODE_NONE = 2;

    /** Which projection to use. Field-tunable for A/B testing. */
    public static int projectionMode = MODE_MEASURED_ACCEL;

    // ---- Braking override: RevAmped's Lambert model -------------------------
    // When a hard deceleration is detected, swap the projection for
    // RevAmped's LambertApproximator, ported 1:1. Their model projects much
    // further ahead than ball-exit (they tuned 0.5s empirically); per its
    // author the long horizon is what gives the lead any effect, which reads
    // as implicit turret-lag compensation. That is compatible with this robot
    // ONLY because there is no command-rate feedforward here - if one is ever
    // added, the same lag would be corrected twice.

    /** Never use the RevAmped model - always project with the normal path. */
    public static final int REVAMPED_OFF = 0;
    /** Use it only while a brake is detected (latched). Normal teleop. */
    public static final int REVAMPED_AUTO = 1;
    /** Use it for EVERY projection, with no deceleration gating at all.
     *  Intended for autonomous, where Pedro drives the whole path and the
     *  approach to every shooting pose is a controlled decel anyway - there
     *  is no cruising-and-shooting case to protect. Set this from the auto
     *  opmode's init and put it back to REVAMPED_AUTO for teleop. */
    public static final int REVAMPED_ALWAYS = 2;

    /** Which of the three above. Field-tunable. */
    public static int revAmpedMode = REVAMPED_AUTO;

    /** Measured decel (in/s^2, negative) that ENGAGES the braking model. Sits
     *  clear of the -52 float coast and well under the -165 active brake. */
    public static double revAmpedBrakeThreshold = -60.0;

    /** Measured decel (in/s^2, negative) at which the braking model RELEASES.
     *
     *  This is deliberately far from the engage threshold. The two projections
     *  disagree a lot - from 70 in/s, the normal path predicts ~48 in/s left
     *  and a small position shift, the RevAmped path predicts ~1 in/s and an
     *  8.8 in shift - so flipping between them mid-brake steps the aim point
     *  and the turret chases it. Latching means: once a brake is recognised,
     *  the whole stop is treated as one event.
     *
     *  -40 also keeps the model engaged through a release-to-coast (-52), on
     *  the reasoning that a driver who brakes then coasts is still stopping.
     *  Raise toward -10 if you want it to hold on even longer; lower toward
     *  -60 if it should drop out as soon as active braking ends. */
    public static double revAmpedReleaseThreshold = -10.0;

    /** Let the RevAmped path bypass the sotmMinSpeed gate.
     *
     *  sotmMinSpeed exists to reject PHANTOM velocity: spinning in place
     *  makes Pinpoint report a few in/s of translation with a random
     *  direction, and leading on that jitters the turret. A projected speed
     *  out of the RevAmped model is not phantom - it is a model output from
     *  a measured brake, with a direction from real motion - so the gate's
     *  reason for existing does not apply to it.
     *
     *  Note what this does and does not buy. The POSITION shift
     *  (projectedDistance) is applied BEFORE the gate and was never affected.
     *  Only the velocity lead is gated. At RevAmped's 0.5s horizon the model
     *  projects ~1.3 in/s from 70 in/s, so the bypass is worth under an inch.
     *  It matters in the 0.2-0.35s horizon band, where the model gives a real
     *  5-15 in/s that the gate would otherwise throw away. */
    public static boolean revAmpedBypassMinSpeed = true;

    /** True on loops where the RevAmped model actually drove the projection. */
    public static boolean revAmpedActive = false;

    /** EMA coefficient on along-track acceleration while DECELERATING.
     *  Tuned for noise robustness, not tracking speed: at a 0.25s horizon the
     *  projection tolerates lag far better than it tolerates noise. Simulation
     *  worst-case hit rate (7in target, sigma=75 in/s^2 accel noise) was 96%
     *  at 0.05 versus 28% at 0.3. */
    public static double accelFilterAlphaDecel = 0.5;

    /** EMA coefficient while ACCELERATING. Powered acceleration decays as the
     *  robot approaches free speed, so a fast filter overshoots. */
    public static double accelFilterAlphaAccel = 0.3;

    /** Hard ceiling on projected speed, in/s. The drivetrain physically cannot
     *  exceed its free speed (~123 in/s at 15V, ~98 at 12V), so any projection
     *  above this is filter overshoot, not real motion. Without this clamp the
     *  hit rate while accelerating dropped from 100% to 24% in simulation. */
    public static double maxProjectedSpeed = 125.0;

    /** Compensate for the velocity vector rotating during the projection
     *  horizon. Without this a steady turn leaves the lead pointing where the
     *  robot USED to be heading: at 30 deg/s the hit rate on a 7in target fell
     *  to 50%, and with it restored to 99%. */
    public static boolean compensateTurning = true;

    /** Ignore |angular velocity| below this (rad/s) as noise. */
    public static double minAngularVelocity = 0.02;

    /** Clamp |angular velocity| to this (rad/s) so a bad reading cannot swing
     *  the lead wildly. 6 rad/s is about 344 deg/s. */
    public static double maxAngularVelocity = 6.0;

    /** Reject |acceleration| above this (in/s^2) as sensor noise. The
     *  drivetrain is traction-limited near 300-430 in/s^2, so anything much
     *  beyond that is not physically reachable. */
    public static double accelClamp = 400.0;

    /** Scales the acceleration estimate when the robot is SPEEDING UP.
     *
     *  Deceleration is trusted at full strength - it is close to constant at
     *  the traction limit and it is what braking shots depend on. Powered
     *  acceleration is different: it decays hyperbolically toward free speed,
     *  so a filtered value taken now over-predicts the average over the
     *  horizon, and the resulting extra lead is applied on exactly the shots
     *  that are already hardest to get right. maxProjectedSpeed catches the
     *  worst of it, but this knob turns the whole effect down smoothly.
     *
     *  1.0 = old behaviour, 0.0 = ignore acceleration entirely (project as if
     *  holding current speed). Only touches positive values; braking is
     *  unaffected at any setting. */
    public static double positiveAccelGain = 0.5;

    // ---- Velocity-history slope estimator ----------------------------------
    // Replaces Pedro's getAcceleration(), which this robot's own accel logs
    // showed is heavily pre-smoothed: at brake onset the velocity signal
    // responds in ONE loop (a 144-181 in/s^2 slope is visible immediately)
    // while getAcceleration() takes ~0.5-0.6s to reach the true value. The
    // ball is long gone by then. d|speed|/dt over a short trailing window IS
    // the along-track acceleration, from a signal that is both fresh and
    // clean (velocity noise measured at sigma = 0.025 in/s at rest).

    /** Trailing window for the slope, seconds. ~3 samples at 65ms loops,
     *  ~8 at 26ms. Shorter = faster response, noisier estimate. */
    public static double slopeWindowSeconds = 0.15;

    /** Unfiltered along-track acceleration estimate, for logging/telemetry.
     *  Compare against lastAlongTrackAccel to see what the EMA is costing. */
    public static double lastAlongTrackAccelRaw = 0.0;

    /** Ring buffer of recent (time, speed) samples. Sized to cover the
     *  window with margin even at fast loop times. */
    private static final int HIST_N = 12;
    private static final double[] histT = new double[HIST_N];
    private static final double[] histV = new double[HIST_N];
    private static int histHead = 0;
    private static int histCount = 0;

    /** Retained EMA state. Reset by resetProjectionFilter(). */
    private static double filteredAlongTrackAccel = 0.0;

    /** Clear the acceleration filter. Call from opmode init so a stale value
     *  from the previous run cannot leak into the first loops of the next. */
    public static void resetProjectionFilter() {
        filteredAlongTrackAccel = 0.0;
        revAmpedActive = false;
        lastAlongTrackAccel = 0.0;
        lastAlongTrackAccelRaw = 0.0;
        lastProjectedSpeed = 0.0;
        histHead = 0;
        histCount = 0;
    }

    // ---- Existing empirical terms -----------------------------------------

    /** Time-of-flight drag correction. Kept from the original calibration. */
    public static double dragTimeFactor = 1;

    /** @deprecated Unused. The coast model replaced the accel lookahead scalar.
     *  Kept only so existing references elsewhere still compile. */
    @Deprecated
    public static double accelScalar = 0.0;

    /** @deprecated Unused; the RPM path is commented out. Kept for compatibility. */
    @Deprecated
    public static double rpmoffset = 200.0;

    public static double sotmFactor = 1.0;
    public static double sotmOffset = 10;
    public static double verticalShift = 0.0;
    public static double verticalShiftStep = 50.0;

    /** Below this projected speed (in/s) the shot is treated as stationary. */
    public static double sotmMinSpeed = 12.0;

    /** Width of the blend between calibration zones, inches. Replaces the hard
     *  switches, which produced a 13 degree step in commanded entry angle at
     *  x = 136 and could flip frame to frame on velocity noise. */
    public static double zoneBlendWidth = 10.0;

    // ---- Telemetry outputs -------------------------------------------------
    public static double requiredTPS = 0.0;
    public static double requiredRPM = 0.0;
    public static int lastIterations = 0;
    public static boolean lastConverged = false;
    public static double lastFlightTime = 0.0;
    public static double lastProjectedSpeed = 0.0;
    public static double lastAlongTrackAccel = 0.0;

    private static final double G_IN = 32.174 * 12.0;   // in/s^2
    private static final double HOOD_MIN = Math.toRadians(46.0);
    private static final double HOOD_MAX = Math.toRadians(80);

    private static final double CLOSE_ZONE_END = 66.29;
    private static final double FAR_ZONE_START = 136.0;

    // ---- Calibration curves ------------------------------------------------

    /** Target height above the launch point, inches, blended across zones. */
    public static double targetHeight(double range) {
        double poly = 0.0032 * range * range - 0.6653 * range + 66.888;
        if(range <= CLOSE_ZONE_END) return poly + 0.4;
        if (range <= FAR_ZONE_START) return poly+0.75;
        if (range >= FAR_ZONE_START + zoneBlendWidth) return 36.0;
        double f = (range - FAR_ZONE_START) / zoneBlendWidth;
        return poly + (36.0 - poly) * f;
    }

    /** Desired entry angle, radians, blended across zones. */
    public static double entryAngle(double range) {
        double closeDeg = 0.6106 * range - 57.478;
        double midDeg = SCORE_ANGLE;
        double farDeg = -30.0;
        double half = zoneBlendWidth / 2.0;

        double deg;
        if (range < CLOSE_ZONE_END - half) {
            deg = closeDeg;
        } else if (range < CLOSE_ZONE_END + half) {
            double f = (range - (CLOSE_ZONE_END - half)) / zoneBlendWidth;
            deg = closeDeg + (midDeg - closeDeg) * f;
        } else if (range < FAR_ZONE_START) {
            deg = midDeg;
        } else if (range < FAR_ZONE_START + zoneBlendWidth) {
            double f = (range - FAR_ZONE_START) / zoneBlendWidth;
            deg = midDeg + (farDeg - midDeg) * f;
        } else {
            deg = farDeg;
        }
        return Math.toRadians(deg);
    }

    /**
     * Ballistic solve for a static target.
     * @return {hoodAngleRadians, launchSpeedInchesPerSecond}
     */
    public static double[] solveStatic(double range, double height, double entry) {
        if (range <= 1e-6) return new double[]{HOOD_MAX, 0.0};

        double hood = Math.atan(2.0 * height / range - Math.tan(entry));
        if (isNaN(hood)) hood = HOOD_MAX;
        hood = MathFunctions.clamp(hood, HOOD_MIN, HOOD_MAX);

        double denom = 2.0 * Math.pow(Math.cos(hood), 2) * (range * Math.tan(hood) - height);
        if (denom <= 0.0) {
            // Geometry unreachable at this hood angle. Try the steepest hood
            // before giving up, since a higher arc can clear a near target.
            hood = HOOD_MAX;
            denom = 2.0 * Math.pow(Math.cos(hood), 2) * (range * Math.tan(hood) - height);
            if (denom <= 0.0) return new double[]{hood, 0.0};
        }
        double speed = Math.sqrt(G_IN * range * range / denom);
        if (isNaN(speed) || Double.isInfinite(speed)) speed = 0.0;
        return new double[]{hood, speed};
    }

    /** Wrap an angle in degrees into (-180, 180]. */
    public static double normalizeDegrees(double degrees) {
        if (isNaN(degrees) || Double.isInfinite(degrees)) return 0.0;
        degrees = degrees % 360.0;
        if (degrees > 180.0) degrees -= 360.0;
        if (degrees <= -180.0) degrees += 360.0;
        return degrees;
    }

    /** Flight time to a target at the given horizontal range, seconds. */
    public static double flightTime(double range) {
        double[] s = solveStatic(range, targetHeight(range), entryAngle(range));
        double horizontal = s[1] * Math.cos(s[0]);
        if (horizontal < 1e-6) return Double.NaN;
        return dragTimeFactor * (range / horizontal);
    }

    /**
     * @param robotHeading      field heading, radians
     * @param robotToGoalVector turret-to-goal, magnitude in inches
     * @param robotVel          chassis velocity, in/s
     * @param robotAccel        chassis acceleration, in/s^2. Used by
     *                          MODE_MEASURED_ACCEL to project the launch state.
     *                          May be null or zero; the projection degrades to
     *                          constant velocity in that case.
     * @return {requiredTPS, hoodServoPosition, turretHeadingErrorDegrees}
     */
    public static Double[] calculateShotVectorandUpdateHeading(
            double robotHeading, Vector robotToGoalVector, Vector robotVel, Vector robotAccel) {
        return calculateShotVectorandUpdateHeading(
                robotHeading, robotToGoalVector, robotVel, robotAccel, 0.0);
    }

    /**
     * @param robotAngularVelocity chassis yaw rate, rad/s. Used to rotate the
     *                             velocity vector forward through the horizon
     *                             so the lead points where the robot will
     *                             actually be heading at launch. Pass 0 if
     *                             unavailable; turning compensation is skipped.
     */
    public static Double[] calculateShotVectorandUpdateHeading(
            double robotHeading, Vector robotToGoalVector, Vector robotVel,
            Vector robotAccel, double robotAngularVelocity) {

        double x = robotToGoalVector.getMagnitude() - ShooterConstants.PASS_THROUGH_POINT_RADIUS;
        if (x < 1.0) x = 1.0;

        double goalTheta = robotToGoalVector.getTheta();

        // ---- Step 4: project the robot state forward to the LAUNCH INSTANT ---
        // The solver must be fed where the robot WILL be, not where it was when
        // the trigger was pulled. This also gives the servo turret its command
        // early, which is the poor-man's velocity reference of Step 5.
        //
        // The projection horizon is the time from now until the ball actually
        // LEAVES the robot: turret settle + feeder delay. Once the ball is in
        // free flight it no longer cares what the chassis does, so the lead
        // inside the fixed-point loop below is flight time ALONE - the feeder
        // delay is already accounted for by projecting to the launch instant.
        // Double-counting it there would over-lead by v * feederDelay.
        //
        // MODE_MEASURED_ACCEL is the default because this robot's deceleration
        // is not repeatable: a zero-power coast is about 47 in/s^2 while active
        // braking is traction-limited near 300 in/s^2. A single fitted coast
        // curve cannot cover both. Reading the actual acceleration adapts to
        // whichever is happening, at the cost of sensor noise - hence the EMA.
        double currentSpeed = robotVel.getMagnitude();
        double velTheta = robotVel.getTheta();

        double horizon = feederDelay + turretSettleTime;

        // Along-track acceleration from the VELOCITY HISTORY, not from
        // robotAccel. d|v|/dt along the direction of travel IS the along-track
        // acceleration, so no vector projection is needed. Real timestamps are
        // used because this robot's loop time varies (26ms cold, 90ms+ hot) -
        // never assume a fixed dt here.
        double nowSec = System.nanoTime() * 1e-9;
        histT[histHead] = nowSec;
        histV[histHead] = currentSpeed;
        histHead = (histHead + 1) % HIST_N;
        if (histCount < HIST_N) histCount++;

        double aAlongV = 0.0;
        double bestDt = 0.0;
        double oldV = currentSpeed;
        for (int i = 0; i < histCount; i++) {
            int idx = (histHead - 1 - i + 2 * HIST_N) % HIST_N;
            double dt = nowSec - histT[idx];
            if (dt <= slopeWindowSeconds && dt > bestDt) {
                bestDt = dt;
                oldV = histV[idx];
            }
        }
        if (bestDt > 0.03) aAlongV = (currentSpeed - oldV) / bestDt;
        // Stationary: kill stale carryover so one maneuver cannot contaminate
        // the next. Without this the filter holds a decel value for a second
        // or more after the robot has already stopped.
        if (currentSpeed < 2.0) aAlongV = 0.0;
        if (isNaN(aAlongV) || Double.isInfinite(aAlongV)) aAlongV = 0.0;
        // Reject physically impossible readings before they reach the filter.
        if (aAlongV > accelClamp) aAlongV = accelClamp;
        if (aAlongV < -accelClamp) aAlongV = -accelClamp;

        lastAlongTrackAccelRaw = aAlongV;

        // Asymmetric EMA. With the slope estimator the input is already both
        // fresh and clean, so these alphas are FAST (0.5 decel / 0.3 accel) -
        // the heavy smoothing that older builds needed was compensating for
        // Pedro's pre-smoothing lag, and on this input it is pure added delay.
        // Accel stays slower than decel because powered acceleration decays
        // toward free speed, so a fast filter overshoots it.
        if (isNaN(filteredAlongTrackAccel)) filteredAlongTrackAccel = 0.0;
        double alpha = (aAlongV < 0.0) ? accelFilterAlphaDecel : accelFilterAlphaAccel;
        filteredAlongTrackAccel += alpha * (aAlongV - filteredAlongTrackAccel);
        lastAlongTrackAccel = filteredAlongTrackAccel;

        double projectedSpeed;
        double projectedDistance;

        switch (projectionMode) {
            case MODE_MEASURED_ACCEL: {
                double a = filteredAlongTrackAccel;

                // Turn down acceleration, leave deceleration alone. Applied
                // before everything else so the brake latch below - which only
                // tests negative values - is unaffected.
                if (a > 0.0) a *= positiveAccelGain;

                // Hard deceleration detected -> hand the projection to
                // RevAmped's model wholesale. Its own horizon is used, NOT the
                // feederDelay-based one, because that long lookahead is the
                // point of their design.
                // LATCHED, not a bare threshold test: engage on hard decel,
                // then hold until the deceleration has clearly ended. A single
                // threshold chatters when the filtered value sits near it, and
                // every flip steps the aim point between two models that
                // disagree by inches - which is what made the turret oscillate.
                if (revAmpedMode == REVAMPED_OFF || currentSpeed <= 2.0) {
                    // Stopped, or disabled: nothing to project, drop the latch.
                    revAmpedActive = false;
                } else if (revAmpedMode == REVAMPED_ALWAYS) {
                    // Forced on - no decel gating, no latch to chatter.
                    revAmpedActive = true;
                } else if (revAmpedActive) {
                    // Already braking - hold until decel weakens past release.
                    revAmpedActive = a < revAmpedReleaseThreshold;
                } else {
                    // Not braking - need a firm decel to engage.
                    revAmpedActive = a < revAmpedBrakeThreshold;
                }

                if (revAmpedActive) {
                    projectedSpeed = RevAmpedBraking.speedAfter(currentSpeed);
                    projectedDistance = RevAmpedBraking.distanceAfter(currentSpeed);
                    break;
                }

                double vEnd = currentSpeed + a * horizon;
                if (vEnd < 0.0) {
                    // The robot stops partway through the horizon. Integrate
                    // only up to the stop instead of letting speed go negative.
                    double tStop = (a < -1e-9) ? (currentSpeed / -a) : 0.0;
                    if (tStop < 0.0) tStop = 0.0;
                    if (tStop > horizon) tStop = horizon;
                    projectedSpeed = 0.0;
                    projectedDistance = currentSpeed * tStop + 0.5 * a * tStop * tStop;
                } else {
                    // Clamp to physical free speed. Filter lag during powered
                    // acceleration otherwise predicts speeds the drivetrain
                    // cannot reach, which over-leads badly.
                    if (vEnd > maxProjectedSpeed) vEnd = maxProjectedSpeed;
                    projectedSpeed = vEnd;
                    double vAvg = 0.5 * (currentSpeed + vEnd);
                    projectedDistance = vAvg * horizon;
                }
                break;
            }
            case MODE_NONE:
            default: {
                projectedSpeed = currentSpeed;
                projectedDistance = currentSpeed * horizon;
                break;
            }
        }

        if (isNaN(projectedSpeed) || projectedSpeed < 0.0) projectedSpeed = 0.0;
        if (isNaN(projectedDistance) || projectedDistance < 0.0) projectedDistance = 0.0;
        lastProjectedSpeed = projectedSpeed;

        // Rotate the velocity direction forward through the horizon. The ball
        // inherits the chassis velocity AT LAUNCH, not now, and on a curved
        // path those differ by omega*horizon. Using the stale direction points
        // the lead where the robot used to be going.
        double omega = robotAngularVelocity;
        if (isNaN(omega) || Double.isInfinite(omega)) omega = 0.0;
        if (!compensateTurning || Math.abs(omega) < minAngularVelocity) omega = 0.0;
        if (omega > maxAngularVelocity) omega = maxAngularVelocity;
        if (omega < -maxAngularVelocity) omega = -maxAngularVelocity;

        // Direction the robot will be travelling when the ball leaves.
        double velThetaLaunch = velTheta + omega * horizon;
        // Mean direction over the horizon, used for the displacement chord.
        double velThetaMid = velTheta + omega * horizon * 0.5;

        // Shift the goal vector to account for the ground covered during the
        // projection window. Working in a frame where +X points at the goal.
        double coastAlongGoal = projectedDistance * Math.cos(velThetaMid - goalTheta);
        double coastPerpGoal = projectedDistance * Math.sin(velThetaMid - goalTheta);

        double projX = x - coastAlongGoal;
        double projY = -coastPerpGoal;
        double projectedRange = Math.hypot(projX, projY);
        if (projectedRange < 1.0) projectedRange = 1.0;
        // Angle from the projected robot position to the goal, relative to the
        // present goal bearing.
        double projectedBearingShift = Math.atan2(projY, projX);

        boolean sotmActive = projectedSpeed >= sotmMinSpeed
                || (revAmpedActive && revAmpedBypassMinSpeed);

        // Velocity components in the projected goal frame, using the direction
        // the robot will actually be travelling at launch.
        double coordinateTheta = velThetaLaunch - (goalTheta + projectedBearingShift);
        double parallelComponent = sotmActive ? Math.cos(coordinateTheta) * projectedSpeed : 0.0;
        double perpendicularComponent = sotmActive ? Math.sin(coordinateTheta) * projectedSpeed : 0.0;

        // ---- Step 3: bounded fixed-point solve ------------------------------
        //   virtual   <- projected_target - lead_gain * projected_velocity * t_f
        //   next_time <- flight_time(distance(shooter, virtual))
        //   stop when |next_time - t_f| < tolerance, or at maxIterations
        // The lesson's t_d appears in the projection horizon above, not here.
        double tf = flightTime(projectedRange);
        if (isNaN(tf) || tf <= 0.0) tf = 0.5;   // seed if the model fails

        double virtualRange = projectedRange;
        double virtualX = projectedRange;
        double virtualY = 0.0;
        int iterations = 0;
        boolean converged = false;

        for (int i = 0; i < maxIterations; i++) {
            iterations = i + 1;

            // Lead is flight time only. The feeder delay was already consumed
            // by projecting the launch state forward through the coast above.
            // Virtual target: displaced opposite the robot's motion.
            virtualX = projectedRange - leadGain * parallelComponent * tf;
            virtualY = -leadGain * perpendicularComponent * tf;
            virtualRange = Math.hypot(virtualX, virtualY);
            if (virtualRange < 1.0) virtualRange = 1.0;

            double nextTf = flightTime(virtualRange);
            if (isNaN(nextTf) || nextTf <= 0.0) break;

            if (Math.abs(nextTf - tf) < solveTolerance) {
                tf = nextTf;
                converged = true;
                break;
            }
            tf = nextTf;
        }
        lastIterations = iterations;
        lastConverged = converged;
        lastFlightTime = tf;

        // ---- Final ballistic solution at the virtual target ------------------
        double[] sol = solveStatic(virtualRange, targetHeight(virtualRange), entryAngle(virtualRange));
        double hoodAngle = sol[0];
        double launchSpeed = sol[1];

        double ballSpeedMps = launchSpeed / 39.37;

        // ---- Turret heading --------------------------------------------------
        // Bearing to the virtual target, relative to the present goal bearing.
        double virtualBearing = goalTheta + projectedBearingShift + Math.atan2(virtualY, virtualX);
        double headingAngle = normalizeDegrees(Math.toDegrees(virtualBearing - robotHeading));

        // sotmFactor scales the lead for empirical trim without touching the
        // solver. At 1.0 the geometric solution is used unmodified.
        if (sotmActive && sotmFactor != 1.0) {
            double baseBearing = normalizeDegrees(Math.toDegrees(goalTheta - robotHeading));
            double leadDelta = normalizeDegrees(headingAngle - baseBearing);
            headingAngle = normalizeDegrees(baseBearing + sotmFactor * leadDelta);
        }

        // ---- Flywheel command -------------------------------------------------
        double tps = (-16.19 * ballSpeedMps * ballSpeedMps + 449.11 * ballSpeedMps - 964.9)
                + verticalShift;
        if (sotmActive) tps -= sotmOffset;

        // The TPS regression is a downward parabola: it turns over near
        // 13.9 m/s and goes negative below about 2.3 m/s. Guard both rails so a
        // bad solve cannot command a nonsense wheel speed.
        if (isNaN(tps) || Double.isInfinite(tps) || launchSpeed <= 0.0) tps = 0.0;
        if (tps < 0.0) tps = 0.0;

        requiredTPS = tps;
        requiredRPM = tps * 60.0 / 28.0;

        double hoodDegrees = Math.toDegrees(hoodAngle);
        double hoodTime = (0.01625 * hoodDegrees) - 0.6;
        if (isNaN(hoodTime)) hoodTime = 0.0;

        ActiveOpMode.telemetry().addData("range (now)", x);
        ActiveOpMode.telemetry().addData("range (projected)", projectedRange);
        ActiveOpMode.telemetry().addData("range (virtual)", virtualRange);
        ActiveOpMode.telemetry().addData("speed now / projected",
                currentSpeed + " / " + projectedSpeed);
        ActiveOpMode.telemetry().addData("flightTime", tf);
        ActiveOpMode.telemetry().addData("solver iters / converged",
                iterations + " / " + converged);
        ActiveOpMode.telemetry().addData("aAlongV (filt)", filteredAlongTrackAccel);
        ActiveOpMode.telemetry().addData("revAmpedActive", revAmpedActive);
        ActiveOpMode.telemetry().addData("revAmpedMode", revAmpedMode);
        ActiveOpMode.telemetry().addData("projMode", projectionMode);
        ActiveOpMode.telemetry().addData("sotmActive", sotmActive);
        ActiveOpMode.telemetry().addData("headingAngle", headingAngle);

        return new Double[]{tps, hoodTime, headingAngle};
    }
}