package org.firstinspires.ftc.teamcode.vision;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public class KalmanFilter {

    private double x;
    private double y;

    // Error covariances for position estimation
    private double px;
    private double py;

    // Process noise (trust in odometry) and Measurement noise (trust in Limelight)
    private static double q = 0.05;
    private static double r = 0.3;

    private Pose lastOdomPose;

    public KalmanFilter(Pose startPose, double q, double r) {
        this.x = startPose.getX();
        this.y = startPose.getY();
        this.lastOdomPose = startPose;

        // Initialize state error estimation higher to allow fast initial convergence
        this.px = 1.0;
        this.py = 1.0;

        KalmanFilter.q = q;
        KalmanFilter.r = r;
    }

    /**
     * Prediction step driven by odometry (e.g., Pedro Pathing wheel encoders / Pinpoint tracker).
     * Call this every single loop cycle with the raw follower/odometry pose.
     */
    public void predict(Pose currentOdomPose) {
        if (currentOdomPose == null) return;

        // Calculate relative displacement since the last loop iteration
        double deltaX = currentOdomPose.getX() - lastOdomPose.getX();
        double deltaY = currentOdomPose.getY() - lastOdomPose.getY();

        // Project the state estimate forward
        x += deltaX;
        y += deltaY;

        // Accumulate uncertainty over time because dead-wheel tracking drifts
        px += q;
        py += q;

        // Cache for the next prediction step
        lastOdomPose = currentOdomPose;
    }

    /**
     * Correction/Update step driven by global AprilTag calculations from the Limelight Subsystem.
     * Call this conditionally when limelight.canSeeTarget() returns true.
     *
     * @param llPose The absolute robot position calculated via Limelight getPose()
     * @param distance The real-time line-of-sight distance to the target via Limelight getDistanceInches()
     */
    public void correct(Pose llPose, double distance) {
        if (llPose == null) return;

        // Dynamically scale measurement noise based on target distance
        // (Camera readings degrade geometrically the further you move away)
        double scale = distance / 70.0;
        double rAdjusted = r * (1.0 + scale * scale);

        // Put a ceiling on adjusted noise to maintain stability bounds
        rAdjusted = Math.min(rAdjusted, 10.0);

        // Compute real-time Kalman Gains (ratio determining state confidence split)
        double kx = px / (px + rAdjusted);
        double ky = py / (py + rAdjusted);

        // Sanity check outlier rejection: Ignore frames that suggest physically impossible jumps
        double dx = llPose.getX() - x;
        double dy = llPose.getY() - y;
        if (Math.hypot(dx, dy) > 6.0) {
            return; // Reject bad vision frames or sudden jitter artifacts
        }

        // Adjust state estimates closer toward the verified global vision position
        x += kx * dx;
        y += ky * dy;

        // Collapse state error uncertainty because absolute global ground-truth was acquired
        px *= (1.0 - kx);
        py *= (1.0 - ky);
    }

    public Pose getFusedPose(double heading) {
        return new Pose(x, y, heading);
    }

    public void setPose(Pose pose) {
        this.x = pose.getX();
        this.y = pose.getY();
        this.px = 1.0;
        this.py = 1.0;
    }
}