package org.firstinspires.ftc.teamcode.opModes.Auto.RED;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.PoseFactory;



import com.pedropathing.ivy.Command;
import org.firstinspires.ftc.teamcode.ivy.*;
import org.firstinspires.ftc.teamcode.pedroPathing.*;

import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.*;
import static org.firstinspires.ftc.teamcode.ivy.HardwareCommands.*;
import static org.firstinspires.ftc.teamcode.opModes.Auto.AutoPathRuntime.*;
import static org.firstinspires.ftc.teamcode.subsystems.DriveTrain2.closeStopperPos;
import static org.firstinspires.ftc.teamcode.subsystems.DriveTrain2.openStopperPos;
import static org.firstinspires.ftc.teamcode.subsystems.DriveTrain2.servoOffset;
import static org.firstinspires.ftc.teamcode.subsystems.ShooterCalcAccelClaude.calculateShotVectorandUpdateHeading;
import static org.firstinspires.ftc.teamcode.subsystems.Flywheel.shooter;

import com.bylazar.configurables.annotations.Configurable;
import org.firstinspires.ftc.teamcode.pedroPathing.TeamFollower;
import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.PolarVector;
import com.pedropathing.paths.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Storage;

import java.util.List;

@Autonomous(name = "Red Far V12")
@Configurable
public class redFar extends IvyOpMode {
    private static final PoseFactory POSES = PoseFactory.radians();

    public redFar() {
        configurePedro(Constants::create);
    }

    private TeamFollower follower;
    private MotorEx transfer;
    private Timer opmodeTimer;
    private Paths paths;

    // Red-alliance start pose, mirrored from blueFar's start POSES.of(42, 9, 90deg)
    // using the same reflection Pose.mirror() applies: x' = 144 - x, y' = y,
    // heading' = normalize(180deg - heading).
    public static double startX = 102;
    public static double startY = 9;

    public Pose start = POSES.of(startX, startY, Math.toRadians(90));

    // --- Turret tracking ---
    private ServoEx servoStopper;
    private ServoEx hoodServo;

    // Goal position mirrored from blueFar's (goalX=2, goalY=140) -> (144-2, 140).
    double goalY = 140;
    double goalX = 140.5;

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

    public static double turretOffset = -3.5;
    public static double turretOffset2 = 2;
    public static double turretOffsetStep = -5;

    // Inches from the Pinpoint/Pedro robot pose origin to the turret pivot.
    // Physical robot geometry constant - not alliance/field dependent, unchanged from blueFar.
    public static double turretForwardOffset = -0.52588;
    public static double turretStrafeOffset = 0;

    private Command intakeMotorOn = Command.build()
            .setStart(() -> {
                intakeMotor.setPower(1);
                transfer.setPower(1);
            });
    private Command farTransfer = Command.build()
            .setStart(() -> {
                intakeMotor.setPower(0.8);
                transfer.setPower(0.8);
            });

    private Command intakeMotorOff = Command.build()
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
        double heading = robotPose.heading();

        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        double turretX = robotPose.x()
                + turretForwardOffset * cos
                - turretStrafeOffset * sin;

        double turretY = robotPose.y()
                + turretForwardOffset * sin
                + turretStrafeOffset * cos;

        return POSES.of(turretX, turretY, heading);
    }

    private PolarVector getTurretToGoalVector(Pose turretPose) {
        return new PolarVector(
                turretPose.distance(POSES.of(goalX, goalY, 0)),
                Math.atan2(goalY - turretPose.y(), goalX - turretPose.x())
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
                .setDone(() -> true);
    }


    // ----------------------

    @Override
    public void onInit() {

        allHubs = RobotContext.hardwareMap().getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        follower = PedroRuntime.follower();

        follower.setStartingPose(start);

        paths = new Paths(follower);

        opmodeTimer = new Timer();

        intakeMotor = new MotorEx("intakeMotor");
        transfer = new MotorEx("transferMotor");

        turret1 = RobotContext.hardwareMap().get(
                ServoImplEx.class,
                "turretServo1");

        turret2 = RobotContext.hardwareMap().get(
                ServoImplEx.class,
                "turretServo2");

        turret1.setPwmRange(new PwmControl.PwmRange(500, 2500));
        turret2.setPwmRange(new PwmControl.PwmRange(500, 2500));
        hoodServo = new ServoEx("hoodServo");

        servoStopper = new ServoEx("stopperServo");

        isOverridden = true;
        preload = true;

        overriddenTurretAngle = getClosestValidTurretAngle(-20);
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

    public Command closeStopper = Command.build()
            .setStart(() -> {
                servoStopper.setPosition(closeStopperPos);
            }).setDone(() -> true);


    public Command openStopper = Command.build()
            .setStart(() -> {
                servoStopper.setPosition(openStopperPos);
            }).setDone(() -> true);

    public Command enableGoalTracking() {
        return Command.build()
                .setStart(() -> {
                    isOverridden = false;
                })
                .setDone(() -> true);
    }
    private Command shoot = sequential(
            waitMs((0.3) * 1000.0),
            openStopper,
            farTransfer,
            waitMs((0.4) * 1000.0),
            intakeMotorOff,
            closeStopper);



    public Command Auto() {
        return sequential(
                disablePreload,
                waitMs((1.0) * 1000.0),
                shoot,
                intakeMotorOn,
                // --- Spike 1 cycle ---
                follow(follower, paths.intakeSpike1, true, 1.0),
                waitMs((0.3) * 1000.0),
                follow(follower, paths.shootSpike1, true, 1.0),
                shoot,
                intakeMotorOn,
                // --- Spike 2 cycle ---
                follow(follower, paths.intakeSpike2, true, 1.0),
                waitMs((0.3) * 1000.0),
                follow(follower, paths.shootSpike2, true, 1.0),
                shoot,
                intakeMotorOn,
                // --- Sweep cycle 1 ---
                follow(follower, paths.intakeSweepHP1, true, 1.0),
                waitMs((0.3) * 1000.0),
                follow(follower, paths.sweepAndShoot1, true, 1.0),
                shoot,
                intakeMotorOn,
                // --- Sweep cycle 2 ---
                follow(follower, paths.intakeSweepHP2, true, 1.0),
                waitMs((0.3) * 1000.0),
                follow(follower, paths.sweepAndShoot2, true, 1.0),
                shoot,
                intakeMotorOn,
                // --- Sweep cycle 3 ---
                follow(follower, paths.intakeSweepHP3, true, 1.0),
                waitMs((0.3) * 1000.0),
                follow(follower, paths.sweepAndShoot3, true, 1.0),
                shoot,
                intakeMotorOn,
                // --- Sweep cycle 4 ---
                follow(follower, paths.intakeSweepHP4, true, 1.0),
                waitMs((0.3) * 1000.0),
                follow(follower, paths.sweepAndShoot4, true, 1.0),
                shoot,
//                intakeMotorOn,
//                // --- Sweep cycle 5 ---
//                follow(follower, paths.intakeSweepHP5, true, 1.0),
//                waitMs((0.3) * 1000.0),
//                follow(follower, paths.sweepAndShoot5, true, 1.0),


                follow(follower, paths.park, true, 1.0)
        );
    }

    public void onStartButtonPressed() {
        opmodeTimer.resetTimer();
        matchStarted = true;
        Auto().schedule();
    }

    private boolean preload = true;

    public Command disablePreload = Command.build()
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

        double robotHeading = follower.getPose().heading();

        PolarVector robotToGoalVector = getTurretToGoalVector(turretPose);

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
            double feedforwardOffset = robotAngularVelocityDegs * 115;
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

        public Path intakeSpike1;
        public Path shootSpike1;
        public Path intakeSpike2;
        public Path shootSpike2;

        public Path intakeSweepHP1;
        public Path hpToShoot1;
        public Path sweepAndShoot1;

        public Path intakeSweepHP2;
        public Path hpToShoot2;
        public Path sweepAndShoot2;

        public Path intakeSweepHP3;
        public Path hpToShoot3;
        public Path sweepAndShoot3;

        public Path intakeSweepHP4;
        public Path hpToShoot4;
        public Path sweepAndShoot4;

        public Path intakeSweepHP5;
        public Path hpToShoot5;
        public Path sweepAndShoot5;

        public Path park;

        // Red-alliance poses, mirrored from blueFar's poses using the same
        // reflection Pose.mirror() applies: x' = 144 - x, y' = y,
        // heading' = normalize(180deg - heading). Two-arg (x, y) poses have no
        // meaningful heading (control points / default 0), so only x is mirrored
        // for those.

        //====Change these only para paths egg=================
        Pose FIRST_SPIKE = POSES.of(120, 30, Math.toRadians(90));
        Pose FIRST_SPIKE_CONTROL = POSES.of(120.5, 17.5, 0);
        Pose FIRST_SHOOT = POSES.of(98, 14, 0);
        Pose FIRST_SHOOT_CONTROL = POSES.of(120.5, 16, 0);
        Pose SECOND_SPIKE = POSES.of(133, 12, Math.toRadians(0));
        Pose SECOND_SHOOT = POSES.of(96.5, 14, 0);
        Pose SWEEP_1 = POSES.of(132.5, 10, Math.toRadians(0));
        Pose SWEEP_2 = POSES.of(132, 14.5, Math.toRadians(60));
        Pose SWEEP_2_CONTROL = POSES.of(127.3, 11.8, 0);
        Pose SWEEP_3 = POSES.of(132, 34.5, Math.toRadians(60));
        Pose SWEEP_SHOOT = POSES.of(86.5, 17.5, 0);
        Pose PARK_POSE = POSES.of(97.5, 22.5, 0);

        public Paths(TeamFollower follower) {

            intakeSpike1 = curve(start, FIRST_SPIKE_CONTROL, FIRST_SPIKE).constant(FIRST_SPIKE.heading());

            shootSpike1 = curve(FIRST_SPIKE, FIRST_SHOOT_CONTROL, FIRST_SHOOT).reverseTangent();

            intakeSpike2 = line(FIRST_SHOOT, SECOND_SPIKE).constant(SECOND_SPIKE.heading());

            shootSpike2 = withCallbacks(
line(SECOND_SPIKE, SECOND_SHOOT).constant(SECOND_SPIKE.heading()),
temporalCallback(150, intakeMotorOff)
);

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

            park = line(SWEEP_SHOOT, PARK_POSE).tangent();
        }

        private Path buildIntakeSweepHP(TeamFollower follower) {
            return line(SECOND_SHOOT, SWEEP_1).constant(SECOND_SPIKE.heading());
        }

        private Path buildHpToShoot(TeamFollower follower) {
            return withCallbacks(
line(SWEEP_1, SECOND_SHOOT).constant(SECOND_SPIKE.heading()),
temporalCallback(150, intakeMotorOff)
);
        }

        private Path buildSweepAndShoot(TeamFollower follower) {
            return withCallbacks(
path(
curve(SWEEP_1, SWEEP_2_CONTROL, SWEEP_2).linear(SWEEP_1.heading(), SWEEP_2.heading()),
line(SWEEP_2, SWEEP_3).constant(SWEEP_2.heading()),
line(SWEEP_3, SWEEP_SHOOT).with(Constants.foresightConfig.brakeAggression.at(0.9)).reverseTangent()
),
temporalCallback(150, intakeMotorOff)
);
        }
    }
}