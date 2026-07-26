package org.firstinspires.ftc.teamcode.opModes.Auto.TRASH;

import static org.firstinspires.ftc.teamcode.subsystems.AutoShooterCalc.calculateShotVectorandUpdateHeading;
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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

@Disabled
@Autonomous
@Configurable
public class Red24BallSpamLinearPivot extends NextFTCOpMode {

    public Red24BallSpamLinearPivot() {
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

    public static double startX = 107.17; // 142 - 33.83
    public static double startY = 134.4;

    public Pose start = new Pose(startX, startY, Math.toRadians(270));

    // --- Turret tracking ---
    private ServoEx servoStopper;
    private ServoEx hoodServo;

    double goalY = 144;
    double goalX = 142;

    public static double gateX = 130.2; // 142 - 11.8
    public static double gateY = 58.85;

    public static double gateX2 = 130.8; // 142 - 11.2
    public static double gateY2 = 58.85;

    public static double gateHeading = 41.25;

    public static double gateX1 = 131.5; // 142 - 10.5
    public static double gateY1 = 58.75;

    public static double turretHeading1 = 169;
    public static double turretHeading2 = 125;
    public static double turretHeading3 = 105;

    public static double gateHeading1 = 42;

    private static final double MIN_ANGLE = -224.75;
    private static final double MAX_ANGLE = 224.75;
    private static final double TURRET_RANGE = 449.51;

    private double currentTurretPos = 90;

    public static double turretHeading4 = -140;

    private boolean matchStarted = false;
    private boolean autoShoot = false;
    private boolean useAutoGoalTracking = true;

    // --- Goal Tracking Override Flags ---
    private boolean isOverridden = false;
    private double overriddenTurretAngle = 0.0;

    private MotorEx intakeMotor;
    private ServoImplEx turret1;
    private ServoImplEx turret2;

    public static double turretOffset = -18;
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

    public Command servoOpen = new LambdaCommand()
            .setStart(() -> {
                servoStopper.setPosition(0.96);
            });

    public Command servoClose = new LambdaCommand()
            .setStart(() -> {
                servoStopper.setPosition(0.86);
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
        double turretX = robotPose.getX()
                + turretForwardOffset * Math.cos(heading)
                - turretStrafeOffset * Math.sin(heading);
        double turretY = robotPose.getY()
                + turretForwardOffset * Math.sin(heading)
                + turretStrafeOffset * Math.cos(heading);

        return new Pose(turretX, turretY, heading);
    }

    private Vector getTurretToGoalVector(Pose turretPose) {
        return new Vector(
                turretPose.distanceFrom(new Pose(goalX, goalY)),
                Math.atan2(goalY - turretPose.getY(), goalX - turretPose.getX())
        );
    }

    //public SequentialGroup shoot = new SequentialGroup(
    //servoOpen,
    //new Delay(0.3)
    //servoClose
    //);

    double targetTurretAngle;




    public boolean manualTPS = true;






    public Command autoShootEnable(){
        return new LambdaCommand()
                .setStart(()->autoShoot = true);
    }
    public Command turnOffManualtps(){
        return new LambdaCommand()
                .setStart(()->manualTPS=false);
    }
    public Command turnOffPreload(){
        return new LambdaCommand()
                .setStart(()->preload=false);
    }

    // --- Custom Override Tracking Commands ---
    public Command setTurretHeading(double degrees) {
        return new LambdaCommand("Set Turret Heading: " + degrees)
                .setStart(() -> {
                    isOverridden = true;
                    overriddenTurretAngle = getClosestValidTurretAngle(degrees);
                })
                .setIsDone(() -> true);
    }

    public Command enableGoalTracking() {
        return new LambdaCommand("Enable Goal Tracking")
                .setStart(() -> {
                    isOverridden = false;
                })
                .setIsDone(() -> true);
    }

    public static MotorEx flywheel = new MotorEx("launchingmotor");

    public static MotorEx flywheel2 = new MotorEx("launchingmotor2");

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

        isOverridden = true;

        setTurretHeading(turretHeading1).schedule();

        hoodServo = new ServoEx("hoodServo");

        servoStopper = new ServoEx("stopperServo");

        telemetry.addLine("Initialized");
        telemetry.update();
    }
    public Command turnTo(Pose targetPose, double timeoutSeconds) {
        return new LambdaCommand("Turn to " + targetPose.getHeading())
                .setStart(() -> follower.holdPoint(targetPose))
                .setIsDone(() -> false)
                .raceWith(new Delay(timeoutSeconds));
    }
    private boolean manualShooting = true;

    public Command closeStopper = new LambdaCommand()
            .setStart(() -> {
                servoStopper.setPosition(closeStopperPos); // close
            }).setIsDone(() -> true);
    public Command openStopper = new LambdaCommand()
            .setStart(() -> {
                servoStopper.setPosition(openStopperPos); // open
            }).setIsDone(() -> true);

    public Command Auto() {
        return new SequentialGroup(

                setTurretHeading(turretHeading1),

                new FollowPath(paths.Preload, false, 1.0),

                intakeMotorOn,

                openStopper,
                new Delay(0.2),
                closeStopper,

                enableGoalTracking(),
                autoShootEnable(),

                new FollowPath(paths.Spike2, false, 1.0),

                new FollowPath(paths.launcgSpike3, false, 1.0)
        );
    }

    public void onStartButtonPressed() {
        opmodeTimer.resetTimer();
        matchStarted = true;
        shooter(2500);
        Auto().schedule();
    }
    private boolean preload = true;
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
                follower.getVelocity().times(1.0),
                follower.getAcceleration());

        flywheelSpeed = results[0];

        if (preload == true) {

            shooter((float) flywheelSpeed + 50);

        }

        if (preload == false) {

            turretOffset = 17;

            shooter((float) flywheelSpeed - 38);
        }

        double hoodAngle = results[1];

        hoodServo.setPosition(hoodAngle);

        double headingError = results[2];

        double robotAngularVelocityRads = follower.getAngularVelocity();

        double robotAngularVelocityDegs =
                Math.toDegrees(robotAngularVelocityRads);

        double feedforwardOffset =
                robotAngularVelocityDegs * 0.225;

        double targetTurretAngle;

        if (isOverridden) {

            targetTurretAngle =
                    getClosestValidTurretAngle(
                            overriddenTurretAngle - feedforwardOffset);

        } else {

            targetTurretAngle =
                    getClosestValidTurretAngle(
                            headingError
                                    + turretOffset
                                    - feedforwardOffset);
        }

        double servoPositionSignal =
                0.05
                        + ((targetTurretAngle - MIN_ANGLE)
                        / 449.51)
                        * 0.90;

        servoPositionSignal =
                Math.max(0.05,
                        Math.min(0.95, servoPositionSignal));

        turret1.setPosition(servoPositionSignal);
        turret2.setPosition(servoPositionSignal);

        currentTurretPos = targetTurretAngle;

        Pose futurepose = new Pose(
                follower.getPose().getX()
                        + follower.getVelocity().getXComponent() * 0.5,

                follower.getPose().getY()
                        + follower.getVelocity().getYComponent() * 0.3,

                follower.getHeading());

        if (isOverlappingLaunchZone(futurepose)
                && robotToGoalVector.getMagnitude() > 45
                && autoShoot) {

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

        private double toRed(double blueDeg) {
            return Math.toRadians(180.0 - blueDeg);
        }

        public PathChain Preload;
        public PathChain Spike2;
        public PathChain launcgSpike3;
        public PathChain gateIntake1;

        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;
        public PathChain Path10;

        public PathChain Pivot2;
        public PathChain Path11;
        public PathChain Path12;
        public PathChain Path13;
        public PathChain Path14;
        public PathChain Path15;
        public PathChain Path16;

        public Paths(Follower follower) {

            Preload = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(startX, startY),
                            new Pose(93.005, 94.650)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(270),
                            Math.toRadians(240))
                    .build();

            Spike2 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(93.005, 94.650),
                            new Pose(91.48, 59.7),
                            new Pose(123.5, 59.5)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(240),
                            Math.toRadians(20))
                    .build();

            launcgSpike3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(123.5, 59.5),
                                    new Pose(78.854, 69.703)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(46), Math.toRadians(0))
                    .setVelocityConstraint(1.0)
                    .setTValueConstraint(0.8)


                    .build();

            gateIntake1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(78.854, 69.703),
                            new Pose(131, 61.5)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(0),Math.toRadians(5))
                    .build();

            Path16 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(131, 61.5),
                            new Pose(131.2, 61.3)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(5),
                            Math.toRadians(gateHeading1))
                    .build();

            Path5 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(131.2, 61.3),
                            new Pose(80.058, 73.32)))
                    .setLinearHeadingInterpolation(
                            gateHeading1,
                            Math.toRadians(-15))
                    .build();

            Path6 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(80.058, 73.32),
                            new Pose(gateX, gateY)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(-15),
                            gateHeading)
                    .build();

            Path7 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(gateX, gateY),
                            new Pose(80.058, 75)))
                    .setLinearHeadingInterpolation(
                            gateHeading,
                            Math.toRadians(-15))
                    .build();

            Path8 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(80.058, 75),
                            new Pose(gateX, gateY)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(-15),
                            gateHeading)
                    .build();

            Path9 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(gateX, gateY),
                            new Pose(80.058, 75)))
                    .setLinearHeadingInterpolation(
                            gateHeading,
                            Math.toRadians(-15))
                    .build();

            Path10 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(92.967, 83.7111129),
                            new Pose(101.019, 63.28),
                            new Pose(gateX2, gateY2)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(-15),
                            gateHeading)
                    .build();

            Path11 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(gateX2, gateY2),
                            new Pose(93.81, 92)))
                    .setLinearHeadingInterpolation(
                            gateHeading,
                            toRed(215))
                    .build();

            Path12 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(93.81, 92),
                            new Pose(106.914, 66.749),
                            new Pose(gateX2, gateY2)))
                    .setLinearHeadingInterpolation(
                            toRed(215),
                            gateHeading)
                    .build();

            Path13 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(gateX2, gateY2),
                            new Pose(82.0, 104.0)))
                    .setLinearHeadingInterpolation(
                            gateHeading,
                            toRed(90))
                    .build();

            Path14 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(80.058, 75),
                            new Pose(85.809, 79.2375),
                            new Pose(122, 83.8)))
                    .setLinearHeadingInterpolation(
                            toRed(195),
                            toRed(180))
                    .build();

            Path15 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(119.0, 83.8),
                            new Pose(92.967, 83.7111129)))
                    .setConstantHeadingInterpolation(
                            toRed(180))
                    .build();
        }
    }
}