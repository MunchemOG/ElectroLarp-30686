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


@Autonomous(name = "Blue Far V12")
@Configurable
public class blueFar extends NextFTCOpMode {

    public blueFar() {
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

    // Raw blue-alliance start pose, copied directly from FarAutoPathsMTI's START_POSE
    // ColoredDecodePose value (blue is the unmirrored base pose there).
    public static double startX = 42;
    public static double startY = 9;

    public Pose start = new Pose(startX, startY, Math.toRadians(90));

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

    public static double turretOffset = -1;
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
    private Command farTransfer = new LambdaCommand()
            .setStart(() -> {
                intakeMotor.setPower(0.9);
                transfer.setPower(0.9);
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

        overriddenTurretAngle = getClosestValidTurretAngle(20);
        double hoodAngle = 0.2;
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
    private SequentialGroup shoot = new SequentialGroup(
            new Delay(0.3),
            openStopper,
            farTransfer,
            new Delay(0.4),
            intakeMotorOff,
            closeStopper);



    public Command Auto() {
        return new SequentialGroup(
                disablePreload,
                new Delay(2.5),
                // Preload shot happens from the start pose — FarAuto.shootPreloads() never
                // drives before shooting, so there's no leading FollowPath here (unlike
                // blue24Near's FollowPath(paths.Preload,...)).
                shoot,
                intakeMotorOn,
                // --- Spike 1 cycle ---
                new FollowPath(paths.intakeSpike1, true, 1.0),
                new Delay(0.3),
                new FollowPath(paths.shootSpike1, true, 1.0),
                shoot,
                intakeMotorOn,
                // --- Spike 2 cycle ---
                new FollowPath(paths.intakeSpike2, true, 1.0),
                new Delay(0.3),
                new FollowPath(paths.shootSpike2, true, 1.0),
                shoot,
                intakeMotorOn,
                // --- Sweep cycle 1 ---
                new FollowPath(paths.intakeSweepHP1, true, 1.0),
                new Delay(0.3),
                new FollowPath(paths.sweepAndShoot1, true, 1.0),
                shoot,
                intakeMotorOn,
                // --- Sweep cycle 2 ---
                new FollowPath(paths.intakeSweepHP2, true, 1.0),
                new Delay(0.3),
                new FollowPath(paths.sweepAndShoot2, true, 1.0),
                shoot,
                intakeMotorOn,
                // --- Sweep cycle 3 ---
                new FollowPath(paths.intakeSweepHP3, true, 1.0),
                new Delay(0.3),
                new FollowPath(paths.sweepAndShoot3, true, 1.0),
                shoot,
                intakeMotorOn,
                // --- Sweep cycle 4 ---
                new FollowPath(paths.intakeSweepHP4, true, 1.0),
                new Delay(0.3),
                new FollowPath(paths.sweepAndShoot4, true, 1.0),
                shoot,
//                intakeMotorOn,
//                // --- Sweep cycle 5 ---
//                new FollowPath(paths.intakeSweepHP5, true, 1.0),
//                new Delay(0.3),
//                new FollowPath(paths.sweepAndShoot5, true, 1.0),

                // No 6th sweep set exists in Paths — see earlier note.

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
            shooter((float) -(flywheelSpeed + 30));
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


        Storage.currentPose = follower.getPose();

        Storage.setPose = true;
    }

    @Override
    public void onStop() {
        Storage.currentPose = follower.getPose();
        follower.breakFollowing();
    }

    public class Paths {

        public PathChain intakeSpike1;
        public PathChain shootSpike1;
        public PathChain intakeSpike2;
        public PathChain shootSpike2;

        public PathChain intakeSweepHP1;
        public PathChain hpToShoot1;
        public PathChain sweepAndShoot1;

        public PathChain intakeSweepHP2;
        public PathChain hpToShoot2;
        public PathChain sweepAndShoot2;

        public PathChain intakeSweepHP3;
        public PathChain hpToShoot3;
        public PathChain sweepAndShoot3;

        public PathChain intakeSweepHP4;
        public PathChain hpToShoot4;
        public PathChain sweepAndShoot4;

        public PathChain intakeSweepHP5;
        public PathChain hpToShoot5;
        public PathChain sweepAndShoot5;

        public PathChain park;

        // Raw blue-alliance poses, copied directly from FarAutoPathsMTI's ColoredDecodePose
        // (blue is unmirrored so lowk wont goon).


        //====Change these only para paths egg=================
        Pose FIRST_SPIKE = new Pose(23.5, 30, Math.toRadians(90));
        Pose FIRST_SPIKE_CONTROL = new Pose(23.5, 16);
        Pose FIRST_SHOOT = new Pose(46, 11.5);
        Pose FIRST_SHOOT_CONTROL = new Pose(23.5, 16);
        Pose SECOND_SPIKE = new Pose(11.5, 12, Math.toRadians(180));
        Pose SECOND_SHOOT = new Pose(47.5, 14);
        Pose SWEEP_1 = new Pose(11.5, 10, Math.toRadians(180));
        Pose SWEEP_2 = new Pose(12, 14.5, Math.toRadians(120));
        Pose SWEEP_2_CONTROL = new Pose(16.7, 11.8);
        Pose SWEEP_3 = new Pose(12, 34.5, Math.toRadians(120));
        Pose SWEEP_SHOOT = new Pose(57.5, 17.5);
        Pose PARK_POSE = new Pose(48, 21);

        public Paths(Follower follower) {

            intakeSpike1 = follower.pathBuilder()
                    .addPath(new BezierCurve(start, FIRST_SPIKE_CONTROL, FIRST_SPIKE))
                    .setConstantHeadingInterpolation(FIRST_SPIKE.getHeading())
                    .build();

            shootSpike1 = follower.pathBuilder()
                    .addPath(new BezierCurve(FIRST_SPIKE, FIRST_SHOOT_CONTROL, FIRST_SHOOT))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            intakeSpike2 = follower.pathBuilder()
                    .addPath(new BezierLine(FIRST_SHOOT, SECOND_SPIKE))
                    .setConstantHeadingInterpolation(SECOND_SPIKE.getHeading())
                    .build();

            shootSpike2 = follower.pathBuilder()
                    .addPath(new BezierLine(SECOND_SPIKE, SECOND_SHOOT))
                    .setConstantHeadingInterpolation(SECOND_SPIKE.getHeading())
                    .addTemporalCallback(150, intakeMotorOff)
                    .build();

            //js goon cycle sweep
            intakeSweepHP1 = buildIntakeSweepHP(follower);
            hpToShoot1 = buildHpToShoot(follower);
            sweepAndShoot1 = buildSweepAndShoot(follower);

            intakeSweepHP2 = buildIntakeSweepHP(follower);
            hpToShoot2 = buildHpToShoot(follower);
            sweepAndShoot2 = buildSweepAndShoot(follower);

            intakeSweepHP3 = buildIntakeSweepHP(follower);
            hpToShoot3 = buildHpToShoot(follower);
            sweepAndShoot3 = buildSweepAndShoot(follower);

            intakeSweepHP4 = buildIntakeSweepHP(follower);
            hpToShoot4 = buildHpToShoot(follower);
            sweepAndShoot4 = buildSweepAndShoot(follower);

            intakeSweepHP5 = buildIntakeSweepHP(follower);
            hpToShoot5 = buildHpToShoot(follower);
            sweepAndShoot5 = buildSweepAndShoot(follower);

            park = follower.pathBuilder()
                    .addPath(new BezierLine(SWEEP_SHOOT, PARK_POSE))
                    .setTangentHeadingInterpolation()
                    .build();
        }

        private PathChain buildIntakeSweepHP(Follower follower) {
            return follower.pathBuilder()
                    .addPath(new BezierLine(SECOND_SHOOT, SWEEP_1))
                    .setConstantHeadingInterpolation(SECOND_SPIKE.getHeading())
                    .build();
        }

        private PathChain buildHpToShoot(Follower follower) {
            return follower.pathBuilder()
                    .addPath(new BezierLine(SWEEP_1, SECOND_SHOOT))
                    .setConstantHeadingInterpolation(SECOND_SPIKE.getHeading())
                    .addTemporalCallback(150, intakeMotorOff)
                    .build();
        }

        private PathChain buildSweepAndShoot(Follower follower) {
            return follower.pathBuilder()
                    .addPath(new BezierCurve(SWEEP_1, SWEEP_2_CONTROL, SWEEP_2))
                    .setLinearHeadingInterpolation(SWEEP_1.getHeading(), SWEEP_2.getHeading())
                    .addPath(new BezierLine(SWEEP_2, SWEEP_3))
                    .setConstantHeadingInterpolation(SWEEP_2.getHeading())
                    .addTemporalCallback(150, intakeMotorOff)
                    .addPath(new BezierLine(SWEEP_3, SWEEP_SHOOT))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();
        }
    }
}