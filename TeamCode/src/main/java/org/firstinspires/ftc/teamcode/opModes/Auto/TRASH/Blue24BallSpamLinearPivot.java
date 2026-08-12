package org.firstinspires.ftc.teamcode.opModes.Auto.TRASH;



import com.pedropathing.ivy.Command;
import org.firstinspires.ftc.teamcode.ivy.*;
import org.firstinspires.ftc.teamcode.pedroPathing.*;

import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.*;
import static org.firstinspires.ftc.teamcode.ivy.HardwareCommands.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.IvyPedroCommands.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.Poses.pose;
import static org.firstinspires.ftc.teamcode.subsystems.AutoShooterCalc.calculateShotVectorandUpdateHeading;
import static org.firstinspires.ftc.teamcode.subsystems.DriveTrain2.closeStopperPos;
import static org.firstinspires.ftc.teamcode.subsystems.DriveTrain2.openStopperPos;
import static org.firstinspires.ftc.teamcode.subsystems.Flywheel.shooter;
import static org.firstinspires.ftc.teamcode.subsystems.LaunchDetector.isOverlappingLaunchZone;

import com.bylazar.configurables.annotations.Configurable;
import org.firstinspires.ftc.teamcode.pedroPathing.TeamFollower;
import org.firstinspires.ftc.teamcode.pedroPathing.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.BezierLine;
import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.PolarVector;
import com.pedropathing.paths.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Storage;

import java.util.List;

@Disabled
@Autonomous
@Configurable
public class Blue24BallSpamLinearPivot extends IvyOpMode {

    public Blue24BallSpamLinearPivot() {
        configurePedro(Constants::create);
    }

    private TeamFollower follower;
    private MotorEx transfer;
    private Timer opmodeTimer;
    private Paths paths;

    public static double startX = 33.83; // 144 - 110.17
    public static double startY = 134.4;
    public Pose start = pose(startX, startY, Math.toRadians(270)); // 180 - 270

    // --- Turret tracking ---
    private ServoEx servoStopper;
    private ServoEx hoodServo;

    double goalY = 144;
    double goalX = 0; // 144 - 144
    public static double gateX = 11.8   ; // 144                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         - 133.2
    public static double gateY = 58.85;

    public static double gateX2 = 11.2;
    public static double gateY2 = 58.85;

    public static double gateHeading = 138.75; // 180 - 41.25

    public static double gateX1 = 10.5; // 144 - 133.5
    public static double gateY1 = 58.75;
    public static double turretHeading1 = -110; // 180 - 60
    public static double turretHeading2 = 125; // 180 - 55
    public static double turretHeading3 = 105; // 180 - 75
    public static double gateHeading1 = 138; // 180 - 42
    private static final double MIN_ANGLE = -224.75;
    private static final double MAX_ANGLE =  224.75;
    private static final double TURRET_RANGE =  449.51;
    private double currentTurretPos = 0;
    public static double turretHeading4  = -140;
    private boolean matchStarted = false;
    private boolean autoShoot = false;
    private boolean useAutoGoalTracking = true;

    // --- Goal Tracking Override Flags ---
    private boolean isOverridden = false;
    private double overriddenTurretAngle = 0.0;

    private MotorEx intakeMotor;
    private ServoImplEx turret1;
    private ServoImplEx turret2;

    public static double turretOffset = -14;
    public static double turretOffsetStep = -5;
    // Inches from the Pinpoint/Pedro robot pose origin to the turret pivot.
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

    public Command servoOpen = Command.build()
            .setStart(() -> {
                servoStopper.setPosition(0.96);
            });

    public Command servoClose = Command.build()
            .setStart(() -> {
                servoStopper.setPosition(0.86);
            });
    private List<LynxModule> allHubs;
    public Pose currPose;

    private static final double ppr = (.225 - .5) / (Math.PI / 2.0);
    public Command setPosition(double position){
        return Command.build()
                .setStart(()->{
                    isOverridden = true;
                    turret1.setPosition(position);
                    turret2.setPosition(position);

                });
    }
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
        double turretX = robotPose.x()
                + turretForwardOffset * Math.cos(heading)
                - turretStrafeOffset * Math.sin(heading);
        double turretY = robotPose.y()
                + turretForwardOffset * Math.sin(heading)
                + turretStrafeOffset * Math.cos(heading);

        return pose(turretX, turretY, heading);
    }

    private PolarVector getTurretToGoalVector(Pose turretPose) {
        return new PolarVector(
                turretPose.distance(pose(goalX, goalY)),
                Math.atan2(goalY - turretPose.y(), goalX - turretPose.x())
        );
    }

    double targetTurretAngle;

    public boolean manualTPS = true;

    public Command autoShootEnable(){
        return Command.build()
                .setStart(()->autoShoot = true);
    }
    public Command turnOffManualtps(){
        return Command.build()
                .setStart(()->manualTPS=false);
    }
    public Command turnOffPreload(){
        return Command.build()
                .setStart(()->preload=false);
    }

    // --- Custom Override Tracking Commands ---
    public Command setTurretHeading(double degrees) {
        return new LambdaCommand("Set Turret Heading: " + degrees)
                .setStart(() -> {
                    isOverridden = true;
                    overriddenTurretAngle = getClosestValidTurretAngle(degrees);
                })
                .setDone(() -> true);
    }

    public boolean shootingCustom = false;
    public Command toggleCustomShooting(boolean value){
        return Command.build()
                .setStart(()->{if(value){
                    shootingCustom = true;
                }
                if(!value){
                    shootingCustom = false;
                }
                });
    }

    public Command enableGoalTracking() {
        return Command.build()
                .setStart(() -> {
                    isOverridden = false;
                })
                .setDone(() -> true);
    }

    public static MotorEx flywheel = new MotorEx("launchingmotor");
    public static MotorEx flywheel2 = new MotorEx("launchingmotor2");

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
        turret1 = RobotContext.hardwareMap().get(ServoImplEx.class, "turretServo1");
        turret2 = RobotContext.hardwareMap().get(ServoImplEx.class,"turretServo2");
        turret1.setPwmRange(new PwmControl.PwmRange(500, 2500));
        turret2.setPwmRange(new PwmControl.PwmRange(500, 2500));

        hoodServo = new ServoEx("hoodServo");
        servoStopper = new ServoEx("stopperServo");
        telemetry.addLine("Initialized");
        telemetry.update();
    }

    public Command turnTo(Pose targetPose, double timeoutSeconds) {
        return new LambdaCommand("Turn to " + targetPose.heading())
                .setStart(() -> follower.holdPoint(targetPose))
                .setDone(() -> false)
                .raceWith(waitMs((timeoutSeconds) * 1000.0));
    }
    private boolean manualShooting = true;

    public Command closeStopper = Command.build()
            .setStart(() -> {
                servoStopper.setPosition(closeStopperPos);
            }).setDone(() -> true);
    public Command openStopper = Command.build()
            .setStart(() -> {
                servoStopper.setPosition(openStopperPos);
            }).setDone(() -> true);

    public Command Auto() {
        return sequential(

                setTurretHeading(turretHeading1),
                waitMs((1.8) * 1000.0),
                follow(follower, paths.Preload, false, 1.0),

                intakeMotorOn,
                openStopper,
                waitMs((0.2) * 1000.0),
                closeStopper,
                enableGoalTracking(),
                autoShootEnable(),
                follow(follower, paths.Spike2, false, 1.0),
                //setTurretHeading(-30),
                follow(follower, paths.launcgSpike3, false, 1.0),
                follow(follower, paths.gateIntake1, true, 1.0),
                follow(follower, paths.Path16,true,1.0),
                waitMs((1.05) * 1000.0),
                //setTurretHeading(-35),

                /*follow(follower, paths.Path5, false, 1.0),

                follow(follower, paths.Path6, true, 1.0),
                waitMs((2.25) * 1000.0),*/
                //setTurretHeading(-25),

                follow(follower, paths.Path7, false, 1.0),

                follow(follower, paths.Path8, true, 1.0),

                waitMs((2.25) * 1000.0),

                follow(follower, paths.Path9, false, 1.0),
                enableGoalTracking(),
                follow(follower, paths.Path14, false, 1.0),
                intakeMotorOff,
                follow(follower, paths.Path15, false, 1.0),
                intakeMotorOn,
                follow(follower, paths.Path10, true, 1.0),
                //follow(follower, paths.Pivot2,false,1.0),
                waitMs((1.05) * 1000.0),
                follow(follower, paths.Path11, false, 1.0),
                follow(follower, paths.Path12, true, 1.0),
                waitMs((2.25) * 1000.0),
                follow(follower, paths.Path13, false, 1.0)
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
        double robotHeading = follower.getPose().heading();
        PolarVector robotToGoalVector = getTurretToGoalVector(turretPose);
        Double[] results = calculateShotVectorandUpdateHeading(robotHeading, robotToGoalVector, follower.getVelocity().times(1.0), follower.getAcceleration());

        flywheelSpeed = results[0];

        if(preload==true&&!shootingCustom){
            shooter((float) flywheelSpeed+30);
//            shooter(2500);
        }
        else if(preload==false&!shootingCustom){
            turretOffset = -17;
            shooter((float) flywheelSpeed - 38);
        }
        else if(shootingCustom){
            turretOffset = -17;
            shooter((float) flywheelSpeed +30);
        }
        double hoodAngle = results[1];
        hoodServo.setPosition(hoodAngle);

        double headingError = results[2];
        double robotAngularVelocityRads = follower.getAngularVelocity();
        double robotAngularVelocityDegs = Math.toDegrees(robotAngularVelocityRads);
        double feedforwardOffset = robotAngularVelocityDegs * 0.225;

        // --- Intercepted for Heading Overrides ---
        double targetTurretAngle;
        if (isOverridden) {
            // Evaluates target angle directly based on user's manual call while preserving feedforward stabilization
            targetTurretAngle = getClosestValidTurretAngle(overriddenTurretAngle - feedforwardOffset);
        } else {
            // Default PolarVector Math Goal Tracking
            targetTurretAngle = getClosestValidTurretAngle(headingError + turretOffset - feedforwardOffset);
        }

        double servoPositionSignal = 0.05 + ((targetTurretAngle - MIN_ANGLE) / 449.51) * 0.90;
        servoPositionSignal = Math.max(0.05, Math.min(0.95, servoPositionSignal));
        turret1.setPosition(servoPositionSignal);
        turret2.setPosition(servoPositionSignal);
        currentTurretPos = targetTurretAngle;
        Pose futurepose = pose(follower.getPose().x()+follower.getVelocity().getXComponent()*0.5, follower.getPose().y()+follower.getVelocity().getYComponent()*0.3, follower.heading());

        if(isOverlappingLaunchZone(futurepose) && robotToGoalVector.getMagnitude()>45&&autoShoot){
            intakeMotor.setPower(1);
            transfer.setPower(1);
            openStopper.schedule();
        }
        else{
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
        public Path Preload;
        public Path Spike2;
        public Path launcgSpike3;
        public Path gateIntake1;
        public Path Path5;
        public Path Path6;
        public Path Path7;
        public Path Path8;
        public Path Path9;
        public Path Path10;

        public Path Pivot2;
        public Path Path11;
        public Path Path12;
        public Path Path13;
        public Path Path14;
        public Path Path15;
        public Path Path16;

        public Paths(TeamFollower follower) {
            Preload = follower.pathBuilder()
                    .addPath(new BezierLine(
                            pose(startX, startY),
                            pose(48.995, 94.650))) // 144 - 95.005
                    .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-60)) // 180-270, 180-240
                    .addPoseCallback(pose(46.255,101.833), autoShootEnable(),0.8 ) // 144 - 97.745
                    .build();

            Spike2 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            pose(48.995, 94.650),
                            pose(50.52, 59.7), // 144 - 93.48
                            pose(18.5, 59.5))) // 144 - 125.5
                    .setLinearHeadingInterpolation(Math.toRadians(-60), Math.toRadians(160)) // 180-240, 180-20
                    .addPoseCallback(pose(45.002,72.527),enableGoalTracking(),0.5)
                    .addPoseCallback(pose(45.002,72.527),turnOffPreload(),0.5)
                    .build();

            launcgSpike3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            pose(18.5, 59.5), // 144 - 125.790
                            pose(63.146, 69.703))) // 144 - 80.854
                    .setLinearHeadingInterpolation(Math.toRadians(160), Math.toRadians(180))// 180-20, 180-0
                    .addPoseCallback(pose(49.376,66.556),enableGoalTracking(),0.78)
                    .build();

            gateIntake1 = follower.pathBuilder() //gateItake
                    .addPath(new BezierLine(
                            pose(63.146, 69.703),
                            pose(14, 61.5))) // 144 - 133
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(175)) // 180-0, 180-5
                    .build();

            Path16 = follower.pathBuilder() //Pivot path
                    .addPath(new BezierLine(
                            pose(14, 61.5),
                            pose(14.2, 61.3))) // 144 - 132.8
                    .setLinearHeadingInterpolation(Math.toRadians(175), Math.toRadians(143)) // 180-5, 180-37
                    .build();

            Path5 = follower.pathBuilder() //shoot
                    .addPath(new BezierLine(
                            pose(14.2, 62.5), // 144 - 132.5
                            pose(61.942, 73.32))) // 144 - 82.058
                    .setLinearHeadingInterpolation(Math.toRadians(gateHeading1), Math.toRadians(195)) // 180 - (-15)
                    .addPoseCallback(pose(48.435,70.259),enableGoalTracking(),0.78)
                    .addPoseCallback(pose(48.435,70.529),intakeMotorOff, 0.78)
                    .build();

            Path6 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            pose(61.942, 73.32),
                            pose(gateX, gateY)))
                    .setLinearHeadingInterpolation(Math.toRadians(195), Math.toRadians(gateHeading))
                    .build();

            Path7 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            pose(gateX, gateY),
                            pose(61.942, 75))) // 144 - 82.100
                    .setLinearHeadingInterpolation(Math.toRadians(gateHeading), Math.toRadians(195))

                    .build();

            Path8 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            pose(61.942, 75),
                            pose(gateX, gateY)))
                    .setLinearHeadingInterpolation(Math.toRadians(195), Math.toRadians(gateHeading))
                    .build();

            Path9 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            pose(gateX, gateY),
                            pose(61.942, 75)))
                    .setLinearHeadingInterpolation(Math.toRadians(gateHeading), Math.toRadians(195))
                    .build();

            Path10 = follower.pathBuilder() // gateintake
                    .addPath(new BezierCurve(
                                pose(49.03304037608623, 83.71111297536585), // 144 - 94.9669...
                            pose(40.981, 63.28),  // 144 - 103.019
                            pose(gateX2, gateY2)))
                    .setLinearHeadingInterpolation(Math.toRadians(195), Math.toRadians(gateHeading))
                    .addPoseCallback(pose(30.424,64.396),toggleCustomShooting(false),0.7)// 180-0
                    .build();
            /*Pivot2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            pose(14.86, 62.8),
                            pose(14.96, 62.5)))
                    .setLinearHeadingInterpolation(Math.toRadians(175),Math.toRadians(143))
                    .build();*/


            Path11 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            pose(gateX2, gateY2),
                            pose(48.19, 92))) // 144 - 95.81
                    .setLinearHeadingInterpolation(Math.toRadians(gateHeading), Math.toRadians(215)) // 180 - (-35)
                    .build();

            Path12 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            pose(48.19, 92),
                            pose(35.086, 66.749), // 144 - 108.914
                            pose(gateX2, gateY2)))
                    .setLinearHeadingInterpolation(Math.toRadians(215), Math.toRadians(gateHeading))
                    .build();

            Path13 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            pose(gateX2, gateY2),
                            pose(60.0, 104.0))) // 144 - 84
                    .setLinearHeadingInterpolation(Math.toRadians(gateHeading), Math.toRadians(90)) // 180-90
                    .build();

            Path14 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            pose(61.942, 75), // 144 - 82
                            pose(56.19133149606427, 79.2375828716257), // 144 - 87.8086...
                            pose(20, 83.8))) // 144 - 121
                    .setLinearHeadingInterpolation(Math.toRadians(195), Math.toRadians(180))
                    .addPoseCallback(pose(44.224,80.162),toggleCustomShooting(true),0.6)
                    .build();

            Path15 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            pose(23.0, 83.8),
                            pose(49.03304037608623, 83.71111297536585)))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();
        }
    }
}