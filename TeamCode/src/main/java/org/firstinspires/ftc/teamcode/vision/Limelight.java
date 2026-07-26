package org.firstinspires.ftc.teamcode.vision;

import static org.firstinspires.ftc.teamcode.opModes.TeleOp.TeleOpBlue2.isBlue;
import static org.firstinspires.ftc.teamcode.opModes.TeleOp.TeleOpRed2.isRed;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.nextftc.core.subsystems.Subsystem;

public class Limelight implements Subsystem {
    private final Limelight3A limelight;
    private final int pipeline;
    private LLResult result;
    double lastDistance;
    private boolean canSeeTarget;

    private static final double LL_HEIGHT_INCHES = 12.8;
    private static final double GOAL_HEIGHT_INCHES = 29.5;
    private static final double LL_ANGLE_DEGREES = 11;
    private static final double METER_TO_INCH = 39.37007;

    public Limelight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(60);

        //1 = red, -1 = blue, 0 = error
        int alliance;
        if (isRed()) {
            alliance = 1;
        } else if (isBlue()) {
            alliance = -1;
        } else {
            alliance = 0;
        }

        switch (alliance) {
            case 1:
                pipeline = 1;
                break;
            case -1:
            default:
                pipeline = 2;
                break;
        }
    }

    @Override
    public void initialize() {
        limelight.pipelineSwitch(pipeline);
    }

    public void start() {
        limelight.start();
    }

    public void stop() {
        limelight.stop();
    }

    /**
     * NextFTC life cycle method. This runs repeatedly every loop
     * cycle, replacing your manual update() method.
     */
    @Override
    public void periodic() {
        result = limelight.getLatestResult();
        canSeeTarget = (result != null && result.isValid());
    }

    public double getDistanceInches() {
        if (canSeeTarget) {
            double tagVerticalAngleDegrees = result.getTy();
            double totalAngleRadians = Math.toRadians(LL_ANGLE_DEGREES + tagVerticalAngleDegrees);
            lastDistance = (GOAL_HEIGHT_INCHES - LL_HEIGHT_INCHES) / Math.tan(totalAngleRadians);
            return lastDistance;
        }
        return -1;
    }

    public Pose getPose(double heading) {
        limelight.updateRobotOrientation(heading);
        if (canSeeTarget) {
            org.firstinspires.ftc.robotcore.external.navigation.Pose3D botPose = result.getBotpose();
            if (botPose != null) {
                double y = -(botPose.getPosition().x) * METER_TO_INCH + 70.5;
                double x = (botPose.getPosition().y) * METER_TO_INCH + 70.5;
                return new Pose(x, y);
            }
        }
        return null;
    }

    public boolean canSeeTarget() {
        return canSeeTarget;
    }
}