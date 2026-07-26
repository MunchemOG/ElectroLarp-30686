package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class ColorBlobDetector implements VisionProcessor {

    public enum Alliance { RED, BLUE }
    public Alliance currentAlliance = Alliance.RED; // Default

    public static class BlobData {
        public boolean found = false;
        public boolean targetLocked = false;
        public double chassisHeadingError = 0;
        public boolean inIntakeRange = false;

        // NEW: Collision Safety Data
        public boolean collisionRisk = false;
        public double avoidanceTurn = 0; // Degrees to turn to avoid hit
    }

    private volatile BlobData latestData = new BlobData();
    private final Mat hsvMat = new Mat();
    private final Mat sampleMask = new Mat();
    private final Mat opponentMask = new Mat();
    private Rect lastBestSample = null;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {}

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);

        // ------------------------------------------------
        // 1. STANDARD SAMPLE TRACKING (Balls/Samples)
        // ------------------------------------------------
        processSampleDetection(frame);

        // ------------------------------------------------
        // 2. OPPONENT ROBOT DETECTION (Collision Avoidance)
        // ------------------------------------------------
        // If we are RED, we look for BLUE robots (and vice versa)
        Scalar lower, upper;
        if (currentAlliance == Alliance.RED) {
            // Look for BLUE Opponents
            lower = new Scalar(100, 100, 50);
            upper = new Scalar(130, 255, 255);
        } else {
            // Look for RED Opponents
            lower = new Scalar(0, 100, 50);
            upper = new Scalar(10, 255, 255);
        }

        Core.inRange(hsvMat, lower, upper, opponentMask);

        // Find opponent contours
        List<MatOfPoint> oppContours = new ArrayList<>();
        Imgproc.findContours(opponentMask, oppContours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        double maxOpponentArea = 0;
        Rect opponentRect = null;

        for (MatOfPoint cnt : oppContours) {
            double area = Imgproc.contourArea(cnt);
            // Opponent robots are HUGE compared to samples
            if (area > 3000) {
                if (area > maxOpponentArea) {
                    maxOpponentArea = area;
                    opponentRect = Imgproc.boundingRect(cnt);
                }
            }
        }

        // Calculate Avoidance Vector
        if (opponentRect != null) {
            // If the opponent is taking up more than 15% of the screen, they are close
            double screenFill = maxOpponentArea / (frame.width() * frame.height());

            if (screenFill > 0.15) {
                latestData.collisionRisk = true;

                double centerX = opponentRect.x + opponentRect.width / 2.0;
                // If opponent is on the RIGHT, we want to dodge LEFT (negative turn)
                // If opponent is on the LEFT, we want to dodge RIGHT (positive turn)
                double positionFromCenter = centerX - (frame.width() / 2.0);

                // Inverse relationship: Push away from the blob
                latestData.avoidanceTurn = -(positionFromCenter * 0.1);
            } else {
                latestData.collisionRisk = false;
                latestData.avoidanceTurn = 0;
            }
        } else {
            latestData.collisionRisk = false;
        }

        opponentMask.release();
        return latestData;
    }

    private void processSampleDetection(Mat frame) {
        // ... (Insert your existing Yellow/Purple/Green detection code here) ...
        // Ensure you populate latestData.found, latestData.targetLocked etc.
    }

    @Override
    public void onDrawFrame(Canvas canvas, int w, int h, float scaleBmp, float scaleDens, Object userContext) {
        // Draw Red Box if Collision Risk
        if (latestData.collisionRisk) {
            Paint p = new Paint();
            p.setColor(Color.RED);
            p.setStyle(Paint.Style.STROKE);
            p.setStrokeWidth(20);
            canvas.drawRect(0, 0, w * scaleBmp, h * scaleBmp, p);
        }
    }

    public BlobData getLatestData() { return latestData; }

    public void setAlliance(Alliance alliance) { this.currentAlliance = alliance; }
}