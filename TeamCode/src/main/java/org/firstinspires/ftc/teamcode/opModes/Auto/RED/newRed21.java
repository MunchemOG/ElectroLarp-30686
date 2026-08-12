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
import static org.firstinspires.ftc.teamcode.subsystems.Flywheel.shooter;
import static org.firstinspires.ftc.teamcode.subsystems.LaunchDetector.isOverlappingLaunchZone;
import static org.firstinspires.ftc.teamcode.subsystems.ShooterCalcAccelClaude.calculateShotVectorandUpdateHeading;

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

@Configurable
@Autonomous(name = "Red New V31111")

public class newRed21 extends IvyOpMode {

    private static final PoseFactory POSES = PoseFactory.radians();

    public newRed21() {
        configurePedro(Constants::create);
    }

    private TeamFollower follower;
    private MotorEx transfer;
    private Timer opmodeTimer;
    private Paths paths;


    public static double startX = 112;
    public static double startY = 134;

    public Pose start = POSES.of(startX, startY, Math.toRadians(-90));


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
    public static double turretOffset2 = 0;
    public static double turretOffsetStep = -5;


    public static double turretForwardOffset = -0.52588;
    public static double turretStrafeOffset = 0;

    private Command intakeMotorOn = Command.build()
            .setStart(() -> {
                intakeMotor.setPower(1);
                transfer.setPower(1);
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


    public Command setTurretHeading(double degrees) {
        return new LambdaCommand("Set Turret Heading: " + degrees)
                .setStart(() -> {
                    isOverridden = true;
                    overriddenTurretAngle = getClosestValidTurretAngle(degrees);
                })
                .setDone(() -> true);
    }

    boolean autoShoot = false;
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

        openStopper.schedule();
        isOverridden = true;
        preload = true;

        overriddenTurretAngle = getClosestValidTurretAngle(170);
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
        flywheel = new MotorEx("launchingmotor");
        flywheel2 = new MotorEx("launchingmotor2");

        telemetry.addLine("Initialized");
        telemetry.update();
    }
    public static MotorEx flywheel;

    public static MotorEx flywheel2 = new MotorEx("launchingmotor2");
    public Command closeStopper = Command.build()
            .setStart(() -> {
                servoStopper.setPosition(closeStopperPos); // close
            }).setDone(() -> true);
    public Command openStopper = Command.build()
            .setStart(() -> {
                servoStopper.setPosition(openStopperPos); // open
            }).setDone(() -> true);

    public Command enableGoalTracking() {
        return Command.build()
                .setStart(() -> {
                    isOverridden = false;
                })
                .setDone(() -> true);
    }

    public Command Auto() {
        return sequential(

                waitMs((1.1) * 1000.0),
                follow(follower, paths.shootPreloads, true, 1.0),
                intakeMotorOn,
                openStopper,
                waitMs((0.2) * 1000.0),
                closeStopper,
                disablePreload,
                // --- Spike 2 cycle ---
                follow(follower, paths.intakeSpike2, true, 1.0),
                follow(follower, paths.shootSpike2, true, 1.0),
                waitMs((0.05) * 1000.0),

                // --- Gate cycle 1 ---
                follow(follower, paths.gateIntake1, true, 1.0),
                waitMs((1.1) * 1000.0),
                follow(follower, paths.gateShoot1, true, 1.0),
                waitMs((0.05) * 1000.0),

                // --- Gate cycle 2 ---
                follow(follower, paths.gateIntake2, true, 1.0),
                waitMs((2.25) * 1000.0),
                follow(follower, paths.gateShoot2, true, 1.0),
                waitMs((0.05) * 1000.0),


                // --- Spike 1 cycle ---
                follow(follower, paths.intakeSpike1, true, 1.0),
                follow(follower, paths.shootSpike1, true, 1.0),
                waitMs((0.05) * 1000.0),

                // --- Gate cycle 3 ---
                follow(follower, paths.gateIntake3, true, 1.0),
                waitMs((1.1) * 1000.0),
                follow(follower, paths.gateShoot3, true, 1.0),
                waitMs((0.05) * 1000.0),

                // --- Gate cycle 4 ---
                follow(follower, paths.gateIntake4, true, 1.0),
                waitMs((2.25) * 1000.0),
                //follow(follower, paths.gateShoot4, true, 1.0),
                //waitMs((0.3) * 1000.0),

                // --- Gate cycle 5 ---
//                follow(follower, paths.gateIntake5, true, 1.0),
//                waitMs((0.3) * 1000.0),
//                follow(follower, paths.gateShoot5, true, 1.0),
//                waitMs((0.3) * 1000.0),

                //follow(follower, paths.park, true, 1.0)
                follow(follower, paths.lastGateWithPark, true, 1.0)
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
                follower.getVelocity().times(0.8), follower.getAcceleration());

        flywheelSpeed = results[0];

        if (preload == true) {
            shooter(6000);
            turretOffset = -11;

        }

        if (preload == false) {
            shooter((float) flywheelSpeed);
            turretOffset = 12;

        }
        double hoodAngle = results[1];
        hoodServo.setPosition(hoodAngle);
        double headingError = results[2];
        double robotAngularVelocityRads = follower.getAngularVelocity();
        double robotAngularVelocityDegs = Math.toDegrees(robotAngularVelocityRads);
        double feedforwardOffset = robotAngularVelocityDegs * 0;
        targetTurretAngle = getClosestValidTurretAngle(headingError + turretOffset - feedforwardOffset);
        double servoPositionSignal = 0.05 + ((targetTurretAngle - MIN_ANGLE) / 449.51) * 0.90;
        servoPositionSignal = Math.max(0.05, Math.min(0.95, servoPositionSignal));

        turret1.setPosition(servoPositionSignal + servoOffset);
        turret2.setPosition(servoPositionSignal - servoOffset);

        currentTurretPos = targetTurretAngle;

        Pose futurepose = POSES.of(follower.getPose().x() + (follower.getVelocity().getXComponent() * 0.2), follower.getPose().y() + (follower.getVelocity().getYComponent() * 0.2), follower.heading());

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



        public Path shootPreloads;
        public Path intakeSpike1;
        public Path shootSpike1;
        public Path intakeSpike2;
        public Path shootSpike2;

        public Path gateIntake1;
        public Path gateShoot1;
        public Path gateIntake2;
        public Path gateShoot2;
        public Path gateIntake3;
        public Path gateShoot3;
        public Path gateIntake4;
        public Path gateShoot4;
        public Path gateIntake5;
        public Path gateShoot5;

        public Path lastGateWithPark;

        public Path park;

        // Every pose below is the mirror() of the corresponding blue pose in
        // Close24AutoPathsMTI: x' = 144 - x, y' = y, heading' = normalizeAngle(pi - heading).

        Pose GATE_1                      = POSES.of(105, 70, Math.toRadians(-29));
        Pose GATE_2                      = POSES.of(115, 63, Math.toRadians(29));
        Pose GATE_3                      = POSES.of(132.85, 58.5, Math.toRadians(36.5));
        Pose GATE_SHOOT_1                = POSES.of(108, 59, Math.toRadians(-29));
        Pose GATE_SHOOT_2                = POSES.of(88, 79, Math.toRadians(-29));
        Pose PARK_POSE                   = POSES.of(95, 71, 0);


        public Paths(TeamFollower follower) {

            shootPreloads = line(start, POSES.of(92.505, 94.650, 0)).linear(Math.toRadians(270),
                            Math.toRadians(240));

            intakeSpike2 = curve(POSES.of(92.505, 94.650, 0),
                            POSES.of(90.98, 59.7, 0),
                            POSES.of(123, 59.5, 0)).linear(Math.toRadians(240),
                            Math.toRadians(20));

            shootSpike2 = line(POSES.of(123.000, 59.500, 0),
                                    GATE_SHOOT_2).linear(Math.toRadians(20), Math.toRadians(-45));

            intakeSpike1 = line(GATE_SHOOT_2,
                                    POSES.of(123.510, 83.297, 0)).tangent();

            shootSpike1 = line(POSES.of(123.510, 83.297, 0),
                                    GATE_SHOOT_2).linear(Math.toRadians(7), Math.toRadians(-45));





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

            park = line(GATE_SHOOT_2, PARK_POSE).tangent();

            lastGateWithPark = curve(GATE_3, POSES.of(101.464, 77, 0), POSES.of(85, 102, 0)).linear(GATE_3.heading(), Math.toRadians(90));
        }

        private Path buildGateIntake(TeamFollower follower) {
            return path(
line(GATE_SHOOT_2, GATE_1).tangent(),
line(GATE_1, GATE_2).linear(GATE_1.heading(), GATE_2.heading()),
line(GATE_2, GATE_3).constant(GATE_3.heading())
);
        }

        private Path buildGateShoot(TeamFollower follower) {
            return curve(GATE_3, GATE_SHOOT_1, GATE_SHOOT_2).reverseTangent();
        }
    }
}
