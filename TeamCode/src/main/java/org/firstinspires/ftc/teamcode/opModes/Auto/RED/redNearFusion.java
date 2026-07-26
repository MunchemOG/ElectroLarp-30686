package org.firstinspires.ftc.teamcode.opModes.Auto.RED;

import static org.firstinspires.ftc.teamcode.subsystems.DriveTrain2.closeStopperPos;
import static org.firstinspires.ftc.teamcode.subsystems.DriveTrain2.openStopperPos;
import static org.firstinspires.ftc.teamcode.subsystems.DriveTrain2.servoOffset;
import static org.firstinspires.ftc.teamcode.subsystems.Flywheel.shooter;
import static org.firstinspires.ftc.teamcode.subsystems.LaunchDetector.isOverlappingLaunchZone;
import static org.firstinspires.ftc.teamcode.subsystems.ShooterCalcAccel.calculateShotVectorandUpdateHeading;

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


@Autonomous(name = "Red Close 21 Near V25")
@Configurable
public class redNearFusion extends NextFTCOpMode {

    public redNearFusion() {
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

    public static double startX = 112;
    public static double startY = 134;

    public Pose start = new Pose(startX, startY, Math.toRadians(-90));


    private ServoEx servoStopper;
    private ServoEx hoodServo;

    double goalY = 140.5;
    double goalX = 140;

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

    public static double turretOffset = 0;
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
                new Delay(1.1),
                new FollowPath(paths.shootPreloads, true, 1.0),
                intakeMotorOn,
                openStopper,
                new Delay(0.2),
                closeStopper,
                disablePreload,
                // --- Spike 2 cycle ---
                new FollowPath(paths.intakeSpike2, true, 1.0),
                new FollowPath(paths.shootSpike2, true, 1.0),
                new Delay(0.05),

                // --- Gate cycle 1 ---
                new FollowPath(paths.gateIntake1, true, 1.0),
                new Delay(1.1),
                new FollowPath(paths.gateShoot1, true, 1.0),
                new Delay(0.05),

                // --- Gate cycle 2 ---
                new FollowPath(paths.gateIntake2, true, 1.0),
                new Delay(2.25),
                new FollowPath(paths.gateShoot2, true, 1.0),
                new Delay(0.05),


                // --- Spike 1 cycle ---
                new FollowPath(paths.intakeSpike1, true, 1.0),
                new FollowPath(paths.shootSpike1, true, 1.0),
                new Delay(0.05),

                // --- Gate cycle 3 ---
                new FollowPath(paths.gateIntake3, true, 1.0),
                new Delay(1.1),
                new FollowPath(paths.gateShoot3, true, 1.0),
                new Delay(0.05),

                // --- Gate cycle 4 ---
                new FollowPath(paths.gateIntake4, true, 1.0),
                new Delay(2.25),
                //new FollowPath(paths.gateShoot4, true, 1.0),
                //new Delay(0.3),

                // --- Gate cycle 5 ---
//                new FollowPath(paths.gateIntake5, true, 1.0),
//                new Delay(0.3),
//                new FollowPath(paths.gateShoot5, true, 1.0),
//                new Delay(0.3),

                //new FollowPath(paths.park, true, 1.0)
                new FollowPath(paths.lastGateWithPark, true, 1.0)
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
                follower.getVelocity().times(0.8), follower.getAcceleration());

        flywheelSpeed = results[0];

        if (preload == true) {
            shooter(6000);
            turretOffset = -11;

        }

        if (preload == false) {
            shooter((float) flywheelSpeed);
            turretOffset = -12;

        }
        double hoodAngle = results[1];
        hoodServo.setPosition(hoodAngle);
        double headingError = results[2];
        double robotAngularVelocityRads = follower.getAngularVelocity();
        double robotAngularVelocityDegs = Math.toDegrees(robotAngularVelocityRads);
        double feedforwardOffset = robotAngularVelocityDegs * 0;
        targetTurretAngle = getClosestValidTurretAngle(headingError - turretOffset - feedforwardOffset);
        double servoPositionSignal = 0.05 + ((targetTurretAngle - MIN_ANGLE) / 449.51) * 0.90;
        servoPositionSignal = Math.max(0.05, Math.min(0.95, servoPositionSignal));

        turret1.setPosition(servoPositionSignal + servoOffset);
        turret2.setPosition(servoPositionSignal - servoOffset);

        currentTurretPos = targetTurretAngle;

        Pose futurepose = new Pose(follower.getPose().getX() + (follower.getVelocity().getXComponent() * 0.2), follower.getPose().getY() + (follower.getVelocity().getYComponent() * 0.2), follower.getHeading());

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

        public PathChain lastGateWithPark;

        public PathChain park;

        // Every pose below is the mirror() of the corresponding blue pose in
        // Close24AutoPathsMTI: x' = 144 - x, y' = y, heading' = normalizeAngle(pi - heading).

        Pose GATE_1                      = new Pose(105, 70, Math.toRadians(-29));
        Pose GATE_2                      = new Pose(115, 63, Math.toRadians(29));
        Pose GATE_3                      = new Pose(132.85, 58.5, Math.toRadians(36.5));
        Pose GATE_SHOOT_1                = new Pose(108, 59, Math.toRadians(-29));
        Pose GATE_SHOOT_2                = new Pose(88, 79, Math.toRadians(-29));
        Pose PARK_POSE                   = new Pose(95, 71);

        public Paths(Follower follower) {
            shootPreloads = follower.pathBuilder()
                    .addPath(new BezierLine(start, new Pose(92.505, 94.650)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(270),
                            Math.toRadians(240))
                    .build();

            intakeSpike2 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(92.505, 94.650),
                            new Pose(90.98, 59.7),
                            new Pose(123, 59.5)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(240),
                            Math.toRadians(20))
                    .build();

            shootSpike2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(123.000, 59.500),
                                    GATE_SHOOT_2
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(20), Math.toRadians(-45))
                    .build();

            intakeSpike1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    GATE_SHOOT_2,
                                    new Pose(123.510, 83.297)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            shootSpike1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(123.510, 83.297),
                                    GATE_SHOOT_2
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(7), Math.toRadians(-45))
                    .build();





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

            lastGateWithPark = follower.pathBuilder()
                    .addPath(new BezierCurve(GATE_3, new Pose(101.464, 77), new Pose(85, 102)))
                    .setLinearHeadingInterpolation(GATE_3.getHeading(), Math.toRadians(90))
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