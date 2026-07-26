package org.firstinspires.ftc.teamcode.vision;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp(name = "Limelight Relocalization Test", group = "Vision")
public class LimelightRelocalizationTest extends NextFTCOpMode {

    private Limelight limelight;
    private KalmanFilter kalmanFilter;
    
    // Initial pose for the filter (matches standard Pedro start or typical test start)
    private final Pose startPose = new Pose(72, 72, Math.toRadians(90));

    public LimelightRelocalizationTest() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onInit() {
        // Initialize the Limelight subsystem
        limelight = new Limelight(hardwareMap);
        
        // Initialize Kalman Filter with starting pose and standard Q/R values
        // Q = 0.1 (Process Noise / Odometry Trust)
        // R = 0.4 (Measurement Noise / Limelight Trust)
        kalmanFilter = new KalmanFilter(startPose, 0.1, 0.4);

        addComponents(new SubsystemComponent(limelight));

        // Reset follower pose to startPose
        follower.setStartingPose(startPose);
    }

    @Override
    public void onUpdate() {
        // 1. Prediction Step: Feed raw odometry into the Kalman Filter
        Pose currentOdomPose = follower.getPose();
        kalmanFilter.predict(currentOdomPose);

        // 2. Correction Step: If Limelight sees a target, update the filter
        boolean canSee = limelight.canSeeTarget();
        double distance = -1;
        Pose visionPose = null;

        if (canSee) {
            distance = limelight.getDistanceInches();
            visionPose = limelight.getPose(currentOdomPose.getHeading());
            
            if (visionPose != null) {
                kalmanFilter.correct(visionPose, distance);
            }
        }

        // 3. Output Telemetry
        Pose fusedPose = kalmanFilter.getFusedPose(currentOdomPose.getHeading());

        telemetry.addLine("=== LIMELIGHT STATUS ===");
        telemetry.addData("Target Visible", canSee);
        telemetry.addData("Target Distance (in)", distance);
        
        telemetry.addLine("\n=== COORDINATES (PEDRO) ===");
        telemetry.addData("Raw Odom X", currentOdomPose.getX());
        telemetry.addData("Raw Odom Y", currentOdomPose.getY());
        
        if (visionPose != null) {
            telemetry.addData("Vision X", visionPose.getX());
            telemetry.addData("Vision Y", visionPose.getY());
        } else {
            telemetry.addLine("Vision Pose: N/A");
        }

        telemetry.addData("Fused X", fusedPose.getX());
        telemetry.addData("Fused Y", fusedPose.getY());
        
        telemetry.addLine("\n=== SYSTEM INFO ===");
        telemetry.addData("Heading (deg)", Math.toDegrees(currentOdomPose.getHeading()));
        telemetry.update();
    }
}
