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
@Autonomous(name = "Red Near 21 V7 ")
@Configurable
public class red21Near1 extends NextFTCOpMode {

    public red21Near1() {
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

    public static double startX = 107.17;
    public static double startY = 134.4;

    public Pose start = new Pose(startX, startY, Math.toRadians(270));

    // --- Turret tracking ---
    private ServoEx servoStopper;
    private ServoEx hoodServo;

    double goalY = 140;
    double goalX = 141;

    public static double gateX = 130.5; // 142 - 11.8
    public static double gateY = 60;

    public static double gateX2 = 130.5; // 142 - 11.2
    public static double gateY2 = 61;

    public static double PivotPoseX = 110;

    public static double PivotPoseY = 70;

    public static double controlX = 7;

    public static double controlY = -4.5;

    public static double launchX = 80.7613;
    public static double launchY = 87.9072;

    public static double gateHeading = 30;

    public static double gateX1 = 131.5; // 142 - 10.5
    public static double gateY1 = 59;


    public static double gateHeading1 = 30;

    private static final double MIN_ANGLE = -224.75;
    private static final double MAX_ANGLE = 224.75;
    private static final double TURRET_RANGE = 449.51;

    private double currentTurretPos = 90;

    public static double turretHeading4 = -140;

    private boolean matchStarted = false;
    private boolean autoShoot = false;
    private boolean useAutoGoalTracking = true;


    private boolean isOverridden = false;
    private double overriddenTurretAngle;

    private MotorEx intakeMotor;
    private ServoImplEx turret1;
    private ServoImplEx turret2;

    public static double turretOffset = -5;
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
    public boolean shooting = false;
    private double normalizeDegrees(double degrees) {
        while (degrees > 180.0) {
            degrees -= 360.0;
        }
        while (degrees <= -180.0) {
            degrees += 360.0;
        }
        return degrees;
    }
    public Command shootFalse = new LambdaCommand()
            .setStart(()->shooting = false);
    public Command shootForMe = new LambdaCommand()
            .setStart(()->shoot());
    public void shoot() {
        if (shooting == false) {
            shooting = true;
            //SequentialGroup shoot = new SequentialGroup(openStopper, intakeMotorOn, new Delay(0.4), closeStopper, shootFalse);
            //shoot.schedule();
        }
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
    // --- Custom Override Tracking Commands ---
    public Command setTurretHeading(double degrees) {
        return new LambdaCommand("Set Turret Heading: " + degrees)
                .setStart(() -> {
                    isOverridden = true;
                    overriddenTurretAngle = getClosestValidTurretAngle(degrees);
                })
                .setIsDone(() -> true);
    }

    /*public Command enableGoalTracking() {
        return new LambdaCommand("Enable Goal Tracking")
                .setStart(() -> {
                    isOverridden = false;
                })
                .setIsDone(() -> true);
    }*/

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
        hoodServo = new ServoEx("hoodServo");

        servoStopper = new ServoEx("stopperServo");

        isOverridden = true;
        preload = true;



        overriddenTurretAngle = getClosestValidTurretAngle(0);
        double hoodAngle = 0.4;
        hoodServo.setPosition(hoodAngle);
        servoStopper.setPosition(closeStopperPos);
        double robotAngularVelocityRads = follower.getAngularVelocity();
        double robotAngularVelocityDegs = Math.toDegrees(robotAngularVelocityRads);
        double feedforwardOffset = 0;

        targetTurretAngle = getClosestValidTurretAngle(overriddenTurretAngle-turretOffset - feedforwardOffset);
        double servoPositionSignal = 0.05 + ((targetTurretAngle - MIN_ANGLE) / 449.51) * 0.90;
        servoPositionSignal = Math.max(0.05, Math.min(0.95, servoPositionSignal));

        turret1.setPosition(/*servoPositionSignal + servoOffset*/0.5+servoOffset);
        turret2.setPosition(/*servoPositionSignal - servoOffset*/0.5-servoOffset);
        double lastServoPos = servoPositionSignal;


        currentTurretPos = targetTurretAngle;
        //enableGoalTracking();




        telemetry.addLine("Initialized");
        telemetry.update();

        //setTurretHeading(turretHeading1).schedule();



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

    public Command Pivot = new LambdaCommand()
            .setStart(()->{
                follower.turnTo(Math.toRadians(40));
            }).setIsDone(()->true);
    public Command enableGoalTracking() {
        return new LambdaCommand("Enable Goal Tracking")
                .setStart(() -> {
                    isOverridden = false;
                })
                .setIsDone(() -> true);
    }


    public Command Auto() {
        return new SequentialGroup(



                new Delay(0.9),


                new FollowPath(paths.Preload, false, 1.0),
                disablePreload,

                shootForMe,
                new Delay(0.4),

                //autoShootEnable(),



                new FollowPath(paths.Spike2, false, 1.0),
                new FollowPath(paths.launchspike2, false, 1.0),
                shootForMe,
                new Delay(0.4),
                new FollowPath(paths.gateIntake1, true, 1.0),
                new FollowPath(paths.Pivot,true,1.0),
                new Delay(1.05),
                new FollowPath(paths.gateIntake1Launch, false, 1.0),
                shootForMe,
                new Delay(0.4),



                new FollowPath(paths.Path8, true, 1.0),

                new Delay(2.25),

                new FollowPath(paths.Path9, false, 1.0),
                shootForMe,
                new Delay(0.2),

                new FollowPath(paths.Path14, false, 1.0),

                new FollowPath(paths.Path15, false, 1.0),
                shootForMe,
                new Delay(0.2),

                new FollowPath(paths.Path10, true, 1.0),
                //new FollowPath(paths.Pivot2,false,1.0),
                new Delay(1.05),
                new FollowPath(paths.Path11, false, 1.0),
                shootForMe,
                new Delay(0.2),
                new FollowPath(paths.Path12, true, 1.0),
                new Delay(2.25),
                new FollowPath(paths.Path13, false, 1.0),
                shootForMe,
                new Delay(0.2)//prk
        );
    }


    public void onStartButtonPressed() {
        opmodeTimer.resetTimer();
        matchStarted = true;
        Auto().schedule();
    }
    private boolean preload = true;

    public Command disablePreload = new LambdaCommand()
            .setStart(()->preload=false);
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
            shooter((float) flywheelSpeed - 10);
            double robotAngularVelocityRads = follower.getAngularVelocity();
            double robotAngularVelocityDegs = Math.toDegrees(robotAngularVelocityRads);
            double feedforwardOffset = 0;

            targetTurretAngle = getClosestValidTurretAngle(overriddenTurretAngle-turretOffset - feedforwardOffset);
            double servoPositionSignal = 0.05 + ((targetTurretAngle - MIN_ANGLE) / 449.51) * 0.90;
            servoPositionSignal = Math.max(0.05, Math.min(0.95, servoPositionSignal));

            turret1.setPosition(/*servoPositionSignal + servoOffset*/0.48+servoOffset);
            turret2.setPosition(/*servoPositionSignal - servoOffset*/0.48-servoOffset);
            double lastServoPos = servoPositionSignal;


            currentTurretPos = targetTurretAngle;


        }

        if (preload == false) {
            shooter((float) flywheelSpeed-20);
            double hoodAngle = results[1];
            hoodServo.setPosition(hoodAngle);
            double headingError = results[2];
            double robotAngularVelocityRads = follower.getAngularVelocity();
            double robotAngularVelocityDegs = Math.toDegrees(robotAngularVelocityRads);
            double feedforwardOffset = robotAngularVelocityDegs * 0.115;
            targetTurretAngle = getClosestValidTurretAngle(headingError + turretOffset - feedforwardOffset);
            double servoPositionSignal = 0.05 + ((targetTurretAngle - MIN_ANGLE) / 449.51) * 0.90;
            servoPositionSignal = Math.max(0.05, Math.min(0.95, servoPositionSignal));
            Pose futurepose = new Pose(follower.getPose().getX() + (follower.getVelocity().getXComponent() * 0.2), follower.getPose().getY() + (follower.getVelocity().getYComponent() * 0.2), follower.getHeading());
            if (isOverlappingLaunchZone(futurepose) && robotToGoalVector.getMagnitude() > 45) {
                intakeMotor.setPower(1);
                transfer.setPower(1);
                openStopper.schedule();
            } else {
                closeStopper.schedule();
            }

            //turret1.setPosition(/*servoPositionSignal + servoOffset*/0.4+servoOffset);
            //turret2.setPosition(/*servoPositionSignal - servoOffset*/0.4-servoOffset);
            turret1.setPosition(servoPositionSignal + servoOffset);
            turret2.setPosition(servoPositionSignal + servoOffset);

            currentTurretPos = targetTurretAngle;
        }



        Pose futurepose = new Pose(follower.getPose().getX() + (follower.getVelocity().getXComponent() * 0.25), follower.getPose().getY() + (follower.getVelocity().getYComponent() * 0.25), follower.getHeading());

        /*if (isOverlappingLaunchZone(futurepose) && robotToGoalVector.getMagnitude() > 45) {
            intakeMotor.setPower(1);
            transfer.setPower(1);
            openStopper.schedule();
        } else {
            closeStopper.schedule();
        }*/

        Storage.currentPose = follower.getPose();

        Storage.setPose = true;
    }

    @Override
    public void onStop() {
        Storage.currentPose = follower.getPose();
        follower.breakFollowing();
    }

    public class Paths {
//ayush loves anjana and moksh is less chopped than deb am i right? yes im always right heehheheheheheheeheheheheh lwk mithun is angy at me hes kinda mean lol


        //dih
        public PathChain Preload;
        public PathChain Spike2;
        public PathChain launchspike2;
        public PathChain gateIntake1;

        public PathChain gateIntake1Launch;
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
        public PathChain Pivot;

        public Paths(Follower follower) {

            Preload = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(startX, startY),
                            new Pose(92.505, 94.650)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(270),
                            Math.toRadians(50))
                    .build();

            Spike2 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(92.505, 94.650),
                            new Pose(90.98, 59.7),
                            new Pose(123, 59.5)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(240),
                            Math.toRadians(20))
                    .build();

            launchspike2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(123, 59.5),
                                    new Pose(79.558, 73.32)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(20), Math.toRadians(45))
                    .setVelocityConstraint(1.0)
                    .setTValueConstraint(0.8)


                    .build();

            gateIntake1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(79.558, 73.32),
                            new Pose(127.5, 59.8)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(49),Math.toRadians(5))
                    .build();

            Pivot = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(127.5, 59.8),
                            new Pose(127.3, 60.1)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(5),
                            Math.toRadians(gateHeading1))
                    .build();

            gateIntake1Launch = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(127.3, 60.1),
                            new Pose(79.558, 73.32)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(gateHeading),
                            Math.toRadians(49))
                    .build();

            Path6 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(79.558, 73.32),
                            new Pose(gateX-0.3, gateY-1.1)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(49),
                            Math.toRadians(gateHeading))
                    .build();

            Path7 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(gateX-0.3, gateY-1.1),
                            new Pose(79.558, 75)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(gateHeading),
                            Math.toRadians(49))
                    .build();

            Path8 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(79.558, 73.32),
                            new Pose(gateX, gateY-1.5)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(49),
                            Math.toRadians(gateHeading))
                    .build();

            Path9 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(gateX, gateY-1.5),
                            new Pose(79.558, 75)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(gateHeading),
                            Math.toRadians(49))
                    .build();

            Path10 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(83, 83.71111297536585),
                            new Pose(100.519, 63.28),
                            new Pose(gateX2-2, gateY2-1.5)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(45),
                            Math.toRadians(gateHeading ))
                    .build();

            Path11 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(gateX2 -2, gateY2-1.5),
                            new Pose(81.5, 104.0)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(gateHeading ),
                            Math.toRadians(35))
                    //toRed(215))
                    .build();

            Path12 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(93.31, 92),
                            new Pose(106.414, 66.749),
                            new Pose(gateX2-2, gateY2-1.5)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(35),
                            Math.toRadians(gateHeading))
                    .build();

            Path13 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(gateX2-2, gateY2-1.5),
                            new Pose(119,52),
                            new Pose(75, 104.0)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(gateHeading -5),
                            Math.toRadians(90))
                    .build();

            Path14 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(84, 75),
                            new Pose(85.3086685039, 79.2375828716257),
                            new Pose(121.5, 83.8)))
                    .setTValueConstraint(0.8)
                    .setLinearHeadingInterpolation(
                            Math.toRadians(49),
                            Math.toRadians(0))
                    .build();


            Path15 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(118.5, 83.8),
                            new Pose(83, 83.71111297536585)))
                    .setTValueConstraint(0.8)
                    .setConstantHeadingInterpolation(
                            Math.toRadians(45))
                    .build();
        }
    }
}