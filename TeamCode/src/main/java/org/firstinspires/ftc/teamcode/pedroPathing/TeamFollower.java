package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.algorithm.Algorithm;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

import static org.firstinspires.ftc.teamcode.pedroPathing.Poses.pose;

/** Pedro 3 follower with project-specific velocity/acceleration conveniences. */
public class TeamFollower extends Follower {
    private final ForesightConfig foresightConfig;
    private PolarVector acceleration = new PolarVector();
    private Velocity previousVelocity = Velocity.zero();
    private long previousUpdateNanos;

    public TeamFollower(Localizer localizer, Drivetrain drivetrain, Algorithm algorithm,
                        ForesightConfig foresightConfig) {
        super(localizer, drivetrain, algorithm);
        this.foresightConfig = foresightConfig;
    }

    @Override public void update() {
        super.update();
        long now = System.nanoTime();
        Velocity current = velocity();
        if (previousUpdateNanos != 0) {
            double dt = (now - previousUpdateNanos) / 1e9;
            if (dt > 1e-6) {
                acceleration = PolarVector.cartesian(
                        (current.vx - previousVelocity.vx) / dt,
                        (current.vy - previousVelocity.vy) / dt);
            }
        }
        previousVelocity = current;
        previousUpdateNanos = now;
    }

    public PathBuilder pathBuilder() { return new PathBuilder(foresightConfig); }
    public Pose getPose() { return pose(); }
    public PolarVector getVelocity() { return PolarVector.cartesian(velocity().vx, velocity().vy); }
    public PolarVector getAcceleration() { return acceleration; }
    public double getAngularVelocity() { return velocity().omega; }
    public double getHeading() { return pose().heading(); }
    public double heading() { return pose().heading(); }
    public double getTotalHeading() { return pose().heading(); }
    public void setStartingPose(Pose pose) { setPose(pose); }
    public void setHeading(double heading) { setPose(Poses.pose(pose().x(), pose().y(), heading)); }
    public void holdPoint(Pose target) { hold(target); }
    public void turnTo(double heading) { hold(Poses.pose(pose().x(), pose().y(), heading)); }
    public void breakFollowing() { stop(); }
    public void startTeleopDrive() { }
    public void startTeleopDrive(boolean robotCentric) { }
    public void setTeleOpDrive(double forward, double strafe, double turn, boolean robotCentric) {
        manual(forward, strafe, turn);
    }
    public void setPathSpeed(double speed) { foresightConfig.setPathSpeed(speed); }
}
