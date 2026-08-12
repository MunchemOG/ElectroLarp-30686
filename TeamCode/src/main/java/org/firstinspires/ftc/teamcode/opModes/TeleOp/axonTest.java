package org.firstinspires.ftc.teamcode.opModes.TeleOp;



//import static org.firstinspires.ftc.teamcode.subsystems.ShooterCalc.calculateShotVectorandUpdateHeading;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@TeleOp(name = "4 Axon Test")
public class axonTest extends LinearOpMode {
    private static final double NUM1 = 0.0;
    private static final double[] NUM2_PRESETS = {1.0, 0.5, 0.3, 0.2};
    private static final double SECONDS_PER_FULL_SWING = 0.3;

    private ServoImplEx testServo1;
    private ServoImplEx testServo2;
    private final ElapsedTime moveTimer = new ElapsedTime();

    private double num2 = NUM2_PRESETS[0];
    private int presetIndex = 0;

    private double currentTargetPos = NUM1;
    private double lastPos = NUM1;
    private double travelTimeNeeded = 0;

    private boolean headingToNum2 = true;
    private boolean prevCircle = false;
    private List<LynxModule> allHubs;

    @Override
    public void runOpMode(){
        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        testServo1 = hardwareMap.get(ServoImplEx.class, "turretServo1");
        testServo2 = hardwareMap.get(ServoImplEx.class, "turretServo2");
        testServo1.setPwmRange(new PwmControl.PwmRange(500, 2500));
        testServo2.setPwmRange(new PwmControl.PwmRange(500, 2500));
        telemetry.addLine("WARNING! Ensure White Turret Gears are Removed");
        telemetry.addLine("Initialized");
        telemetry.update();
        testServo1.setPosition(NUM1);
        testServo2.setPosition(NUM1);
        currentTargetPos = NUM1;
        lastPos = NUM1;

        telemetry.addLine("ServoTest ready. Press PLAY to start cycling.");
        telemetry.addData("num1", NUM1);
        telemetry.addData("num2", num2);
        telemetry.update();

        waitForStart();

        startMove(num2);

        while (opModeIsActive()) {

            boolean circleNow = gamepad1.circle;
            if (circleNow && !prevCircle) {
                presetIndex = (presetIndex + 1) % NUM2_PRESETS.length;
                num2 = NUM2_PRESETS[presetIndex];
            }
            prevCircle = circleNow;

            if (moveTimer.seconds() >= travelTimeNeeded) {
                if (headingToNum2) {
                    startMove(NUM1);
                    headingToNum2 = false;
                } else {
                    startMove(num2);
                    headingToNum2 = true;
                }
            }

            telemetry.addData("num1 (home)", NUM1);
            telemetry.addData("num2 (target, circle to cycle)", num2);
            telemetry.addData("current commanded pos", currentTargetPos);
            telemetry.addData("heading to num2?", headingToNum2);
            telemetry.addData("move elapsed (s)", "%.2f", moveTimer.seconds());
            telemetry.addData("move needed (s)", "%.2f", travelTimeNeeded);
            telemetry.update();
        }
    }

    private void startMove(double newTarget) {
        double distance = Math.abs(newTarget - lastPos);
        travelTimeNeeded = distance * SECONDS_PER_FULL_SWING;
        testServo1.setPosition(newTarget);
        testServo2.setPosition(newTarget);
        currentTargetPos = newTarget;
        lastPos = newTarget;
        moveTimer.reset();
    }
}
