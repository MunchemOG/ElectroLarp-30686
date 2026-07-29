package org.firstinspires.ftc.teamcode.opModes.Auto.BLUE;

import static org.firstinspires.ftc.teamcode.subsystems.DriveTrain2.servoOffset;
import static org.firstinspires.ftc.teamcode.subsystems.ShooterCalcAccelClaude.calculateShotVectorandUpdateHeading;
import static org.firstinspires.ftc.teamcode.subsystems.DriveTrain2.closeStopperPos;
import static org.firstinspires.ftc.teamcode.subsystems.DriveTrain2.openStopperPos;
import static org.firstinspires.ftc.teamcode.subsystems.Flywheel.shooter;
import static org.firstinspires.ftc.teamcode.subsystems.LaunchDetector.isOverlappingLaunchZone;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Storage;

import java.util.List;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;


@Autonomous(name = "Blue Near V1")
@Configurable
public class blueNear extends NextFTCOpMode {

    public blueNear() {
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(hwMap -> Constants.createFollower(hwMap))
        );
    }

    private Follower follower;
    private MotorEx transfer;
    private Timer opmodeTimer;
    private Paths paths;

    // Raw blue-alliance start pose
    // ColoredDecodePose value (blue is the unmirrored base pose there).
    public static double startX = 32;
    public static double startY = 134;

    public Pose start = new Pose(startX, startY, Math.toRadians(270));

    // --- Turret tracking ---
    private ServoEx servoStopper;
    private ServoEx hoodServo;

    double goalY = 140;
    double goalX = 2;

    private static final double MIN_ANGLE = -224.75;
    private static final double MAX_ANGLE = 224.75;
    private static final double TURRET_RANGE = 449.51;

    private double currentTurretPos = 90;

    private boolean matchStarted = false;
    private boolean useAutoGoalTracking = true;

    private boolean isOverridden = false;
    private double overriddenTurretAngle;

    private MotorEx intakeMotor;
    private ServoImplEx turret1;
    private ServoImplEx turret2;

    public static double turretOffset = 10;
    public static double turretOffset2 = 2;
    public static double turretOffsetStep = -5;

    // Inches from the Pinpoint/Pedro robot pose origin to the turret pivot.
    public static double turretForwardOffset = -0.52588;
    public static double turretStrafeOffset = 0;

    private Command intakeMotorOn = new LambdaCommand()
            .setStart(() -> {
                intakeMotor.setPower(1);
                transfer.setPower(1);
            });

    private Command intakeMotorOff = new LambdaCommand()
            .setStart(() -> {
                intakeMotor.setPower(0);
                transfer.setPower(0);
            });

    private List<LynxModule> allHubs;
    public Pose currPose;

    public double getClosestValidTurretAngle(double relativeGoalDegrees) {
        double option1 = normalizeDegrees(relativeGoalDegrees);
        return Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, option1));
    }

    private double normalizeDegrees(double degrees) {
        while (degrees > 180.0) {
            degrees -= 360.0;
        }
        while (degrees <= -180.0) {
            degrees += 360.0;
        }
        return degrees;
    }

    private Pose getTurretPose(Pose robotPose) {
        double heading = robotPose.getHeading();

        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        double turretX = robotPose.getX()
                + turretForwardOffset * cos
                - turretStrafeOffset * sin;

        double turretY = robotPose.getY()
                + turretForwardOffset * sin
                + turretStrafeOffset * cos;

        return new Pose(turretX, turretY, heading);
    }

    private Vector getTurretToGoalVector(Pose turretPose) {
        return new Vector(
                turretPose.distanceFrom(new Pose(goalX, goalY)),
                Math.atan2(goalY - turretPose.getY(), goalX - turretPose.getX())
        );
    }

    double targetTurretAngle;

    public boolean manualTPS = true;

    // --- Custom Override Tracking Commands ---
    public Command setTurretHeading(double degrees) {
        return new LambdaCommand("Set Turret Heading: " + degrees)
                .setStart(() -> {
                    isOverridden = true;
                    overriddenTurretAngle = getClosestValidTurretAngle(degrees);
                })
                .setIsDone(() -> true);
    }

    // NOTE: no static MotorEx flywheel/flywheel2 fields here on purpose — static hardware
    // fields construct at class-load time (OpMode discovery), before any OpMode is active
    // or hardwareMap exists, which is a known cause of an instant crash on init in NextFTC.

    // ----------------------

    @Override
    public void onInit() {

        allHubs = ActiveOpMode.hardwareMap().getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        follower = PedroComponent.follower();

        follower.setStartingPose(start);

        paths = new Paths(follower);

        opmodeTimer = new Timer();

        intakeMotor = new MotorEx("intakeMotor");
        transfer = new MotorEx("transferMotor");

        turret1 = ActiveOpMode.hardwareMap().get(
                ServoImplEx.class,
                "turretServo1");

        turret2 = ActiveOpMode.hardwareMap().get(
                ServoImplEx.class,
                "turretServo2");

        turret1.setPwmRange(new PwmControl.PwmRange(500, 2500));
        turret2.setPwmRange(new PwmControl.PwmRange(500, 2500));
        hoodServo = new ServoEx("hoodServo");

        servoStopper = new ServoEx("stopperServo");

        isOverridden = true;
        preload = true;

        overriddenTurretAngle = getClosestValidTurretAngle(160);
        double hoodAngle = 0.4;
        hoodServo.setPosition(hoodAngle);
        servoStopper.setPosition(closeStopperPos);
        double robotAngularVelocityRads = follower.getAngularVelocity();
        double robotAngularVelocityDegs = Math.toDegrees(robotAngularVelocityRads);
        double feedforwardOffset = 0;

        targetTurretAngle = getClosestValidTurretAngle(overriddenTurretAngle + turretOffset - feedforwardOffset);
        double servoPositionSignal = 0.05 + ((targetTurretAngle - MIN_ANGLE) / 449.51) * 0.90;
        servoPositionSignal = Math.max(0.05, Math.min(0.95, servoPositionSignal));

        turret1.setPosition(servoPositionSignal + servoOffset);
        turret2.setPosition(servoPositionSignal - servoOffset);
        double lastServoPos = servoPositionSignal;

        currentTurretPos = targetTurretAngle;

        telemetry.addLine("Initialized");
        telemetry.update();
    }

    public Command closeStopper = new LambdaCommand()
            .setStart(() -> {
                servoStopper.setPosition(closeStopperPos); // close
            }).setIsDone(() -> true);
    public Command openStopper = new LambdaCommand()
            .setStart(() -> {
                servoStopper.setPosition(openStopperPos); // open
            }).setIsDone(() -> true);

    public Command enableGoalTracking() {
        return new LambdaCommand("Enable Goal Tracking")
                .setStart(() -> {
                    isOverridden = false;
                })
                .setIsDone(() -> true);
    }


    public Command Auto() {
        return new SequentialGroup(
                new Delay(0.3),
                new FollowPath(paths.shootPreloads, true, 1.0),

                intakeMotorOn,
                openStopper,
                new Delay(0.2),
                closeStopper,
                disablePreload,

                // --- Spike 1 cycle ---
                new FollowPath(paths.intakeSpike1, true, 1.0),
                new FollowPath(paths.shootSpike1, true, 1.0),
                new Delay(0.3),

                // --- Spike 2 cycle ---
                new FollowPath(paths.intakeSpike2, true, 1.0),
                new FollowPath(paths.shootSpike2, true, 1.0),
                new Delay(0.3),

                // --- Gate cycle 1 ---
                new FollowPath(paths.gateIntake1, true, 1.0),
                new Delay(1),
                new FollowPath(paths.gateShoot1, true, 1.0),
                new Delay(0.3),

                // --- Gate cycle 2 ---
                new FollowPath(paths.gateIntake2, true, 1.0),
                new Delay(2.25),
                new FollowPath(paths.gateShoot2, true, 1.0),
                new Delay(0.3),

                // --- Gate cycle 3 ---
                new FollowPath(paths.gateIntake3, true, 1.0),
                new Delay(2.25),
                new FollowPath(paths.gateShoot3, true, 1.0),
                new Delay(0.3),

                // --- Gate cycle 4 ---
                new FollowPath(paths.gateIntake4, true, 1.0),
                new Delay(2.25),
                new FollowPath(paths.gateShoot4, true, 1.0),
                new Delay(0.3),

                // --- Gate cycle 5 ---
//                new FollowPath(paths.gateIntake5, true, 1.0),
//                new Delay(0.3),
//                new FollowPath(paths.gateShoot5, true, 1.0),
//                new Delay(0.3),

                new FollowPath(paths.park, true, 1.0)
        );
    }

    public void onStartButtonPressed() {
        opmodeTimer.resetTimer();
        matchStarted = true;
        Auto().schedule();
    }

    private boolean preload = true;

    public Command disablePreload = new LambdaCommand()
            .setStart(() -> preload = false);
    private double flywheelSpeed;

    @Override
    public void onUpdate() {

        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        follower.update();

        Storage.currentPose = follower.getPose();

        if (!matchStarted) return;

        Pose currPose = follower.getPose();

        Pose turretPose = getTurretPose(currPose);

        double robotHeading = follower.getPose().getHeading();

        Vector robotToGoalVector = getTurretToGoalVector(turretPose);

        Double[] results = calculateShotVectorandUpdateHeading(
                robotHeading,
                robotToGoalVector,
                follower.getVelocity().times(1.0), follower.getAcceleration());

        flywheelSpeed = results[0];

        if (preload == true) {
            double hoodAngle = results[1];
            hoodServo.setPosition(hoodAngle);
            shooter((float) flywheelSpeed + 30);
            double robotAngularVelocityRads = follower.getAngularVelocity();
            double robotAngularVelocityDegs = Math.toDegrees(robotAngularVelocityRads);
            double feedforwardOffset = 0;

            targetTurretAngle = getClosestValidTurretAngle(overriddenTurretAngle - turretOffset - feedforwardOffset);
            double servoPositionSignal = 0.05 + ((targetTurretAngle - MIN_ANGLE) / 449.51) * 0.90;
            servoPositionSignal = Math.max(0.05, Math.min(0.95, servoPositionSignal));

            turret1.setPosition(servoPositionSignal + servoOffset);
            turret2.setPosition(servoPositionSignal - servoOffset);
            double lastServoPos = servoPositionSignal;

            currentTurretPos = targetTurretAngle;

        }

        if (preload == false) {
            shooter((float) flywheelSpeed);
            double hoodAngle = results[1];
            hoodServo.setPosition(hoodAngle);
            double headingError = results[2];
            double robotAngularVelocityRads = follower.getAngularVelocity();
            double robotAngularVelocityDegs = Math.toDegrees(robotAngularVelocityRads);
            double feedforwardOffset = robotAngularVelocityDegs * 0.115;
            targetTurretAngle = getClosestValidTurretAngle(headingError - turretOffset - feedforwardOffset);
            double servoPositionSignal = 0.05 + ((targetTurretAngle - MIN_ANGLE) / 449.51) * 0.90;
            servoPositionSignal = Math.max(0.05, Math.min(0.95, servoPositionSignal));

            turret1.setPosition(servoPositionSignal + servoOffset);
            turret2.setPosition(servoPositionSignal - servoOffset);

            currentTurretPos = targetTurretAngle;
        }

        Pose futurepose = new Pose(follower.getPose().getX() + (follower.getVelocity().getXComponent() * 0.25), follower.getPose().getY() + (follower.getVelocity().getYComponent() * 0.25), follower.getHeading());

        if (isOverlappingLaunchZone(futurepose) && robotToGoalVector.getMagnitude() > 45) {
            intakeMotor.setPower(1);
            transfer.setPower(1);
            openStopper.schedule();
        } else {
            closeStopper.schedule();
        }

        Storage.currentPose = follower.getPose();

        Storage.setPose = true;
    }

    @Override
    public void onStop() {
        Storage.currentPose = follower.getPose();
        follower.breakFollowing();
    }

    public class Paths {

        public PathChain shootPreloads;
        public PathChain intakeSpike1;
        public PathChain shootSpike1;
        public PathChain intakeSpike2;
        public PathChain shootSpike2;

        public PathChain gateIntake1;
        public PathChain gateShoot1;
        public PathChain gateIntake2;
        public PathChain gateShoot2;
        public PathChain gateIntake3;
        public PathChain gateShoot3;
        public PathChain gateIntake4;
        public PathChain gateShoot4;
        public PathChain gateIntake5;
        public PathChain gateShoot5;

        public PathChain park;

        // Raw blue-alliance poses, copied directly from Close24AutoPathsMTI's
        // ColoredDecodePose field values (blue is the unmirrored base pose there,
        // so these numbers are exact, not approximated).
        Pose PRELOADS_SHOOT              = new Pose(32, 98, Math.toRadians(270));
        Pose FIRST_SPIKE_1_CONTROL       = new Pose(24, 98);
        Pose FIRST_SPIKE_1               = new Pose(24, 94, Math.toRadians(270));
        Pose FIRST_SPIKE_2               = new Pose(24, 87, Math.toRadians(270));
        Pose FIRST_SPIKE_SHOOT           = new Pose(46, 84, Math.toRadians(270));
        Pose SECOND_SPIKE_1              = new Pose(46, 53, Math.toRadians(190));
        Pose SECOND_SPIKE_2              = new Pose(40, 58, Math.toRadians(180));
        Pose SECOND_SPIKE_3              = new Pose(18.5, 57, Math.toRadians(180));
        Pose SECOND_SPIKE_SHOOT_CONTROL  = new Pose(37, 66);
        Pose SECOND_SPIKE_SHOOT          = new Pose(56, 80, Math.toRadians(209));
        Pose GATE_1                      = new Pose(39, 70, Math.toRadians(209));
        Pose GATE_2                      = new Pose(29, 63, Math.toRadians(151));
        Pose GATE_3                      = new Pose(11.5, 57.5, Math.toRadians(151));
        Pose GATE_SHOOT_1                = new Pose(36, 59, Math.toRadians(209));
        Pose GATE_SHOOT_2                = new Pose(56, 79, Math.toRadians(209));
        Pose PARK_POSE                   = new Pose(49, 71);

        public Paths(Follower follower) {

            shootPreloads = follower.pathBuilder()
                    .addPath(new BezierLine(start, PRELOADS_SHOOT))
                    .setConstantHeadingInterpolation(PRELOADS_SHOOT.getHeading())
                    .build();

            intakeSpike1 = follower.pathBuilder()
                    .addPath(new BezierCurve(PRELOADS_SHOOT, FIRST_SPIKE_1_CONTROL, FIRST_SPIKE_1))
                    .setConstantHeadingInterpolation(PRELOADS_SHOOT.getHeading())
                    .addPath(new BezierLine(FIRST_SPIKE_1, FIRST_SPIKE_2))
                    .setConstantHeadingInterpolation(FIRST_SPIKE_2.getHeading())
                    .build();

            shootSpike1 = follower.pathBuilder()
                    .addPath(new BezierLine(FIRST_SPIKE_2, FIRST_SPIKE_SHOOT))
                    .setConstantHeadingInterpolation(FIRST_SPIKE_2.getHeading())
                    .build();

            intakeSpike2 = follower.pathBuilder()
                    .addPath(new BezierCurve(FIRST_SPIKE_SHOOT, SECOND_SPIKE_1, SECOND_SPIKE_2, SECOND_SPIKE_3))
                    .setLinearHeadingInterpolation(FIRST_SPIKE_SHOOT.getHeading(), SECOND_SPIKE_2.getHeading(), 0.25)
                    .build();

            shootSpike2 = follower.pathBuilder()
                    .addPath(new BezierCurve(SECOND_SPIKE_3, SECOND_SPIKE_SHOOT_CONTROL, SECOND_SPIKE_SHOOT))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            // Close24AutoPathsMTI builds gateIntake/gateShoot the same way all 5 times —
            // no per-cycle variation exists in the source, so repeating them here is a
            // faithful copy, not a simplification.
            gateIntake1 = buildGateIntake(follower);
            gateShoot1 = buildGateShoot(follower);

            gateIntake2 = buildGateIntake(follower);
            gateShoot2 = buildGateShoot(follower);

            gateIntake3 = buildGateIntake(follower);
            gateShoot3 = buildGateShoot(follower);

            gateIntake4 = buildGateIntake(follower);
            gateShoot4 = buildGateShoot(follower);

            gateIntake5 = buildGateIntake(follower);
            gateShoot5 = buildGateShoot(follower);

            park = follower.pathBuilder()
                    .addPath(new BezierLine(GATE_SHOOT_2, PARK_POSE))
                    .setTangentHeadingInterpolation()
                    .build();
        }

        private PathChain buildGateIntake(Follower follower) {
            return follower.pathBuilder()
                    .addPath(new BezierLine(GATE_SHOOT_2, GATE_1))
                    .setTangentHeadingInterpolation()
                    .addPath(new BezierLine(GATE_1, GATE_2))
                    .setLinearHeadingInterpolation(GATE_1.getHeading(), GATE_2.getHeading())
                    .addPath(new BezierLine(GATE_2, GATE_3))
                    .setConstantHeadingInterpolation(GATE_3.getHeading())
                    .build();
        }

        private PathChain buildGateShoot(Follower follower) {
            return follower.pathBuilder()
                    .addPath(new BezierCurve(GATE_3, GATE_SHOOT_1, GATE_SHOOT_2))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();
        }
    }
}