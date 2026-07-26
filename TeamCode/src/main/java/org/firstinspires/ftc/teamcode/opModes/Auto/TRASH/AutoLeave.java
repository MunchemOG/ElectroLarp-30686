package org.firstinspires.ftc.teamcode.opModes.Auto.TRASH;

import static org.firstinspires.ftc.teamcode.subsystems.AutoShooterCalc.calculateShotVectorandUpdateHeading;
import static org.firstinspires.ftc.teamcode.subsystems.DriveTrain2.closeStopperPos;
import static org.firstinspires.ftc.teamcode.subsystems.DriveTrain2.openStopperPos;
import static org.firstinspires.ftc.teamcode.subsystems.Flywheel.shooter;
import static org.firstinspires.ftc.teamcode.subsystems.LaunchDetector.isOverlappingLaunchZone;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
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
public class AutoLeave extends NextFTCOpMode {

    public AutoLeave() {
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

    public Pose start = new Pose(startX, startY, Math.toRadians(90));

    // --- Turret tracking ---
    private ServoEx servoStopper;
    private ServoEx hoodServo;
    public static double startX = 79.318;
    public static double startY = 9.2;

    private MotorEx intakeMotor;
    private boolean isOverridden = false;
    private double overriddenTurretAngle = 0.0;
    double goalY = 144;
    double goalX = 144;
    public static double gateX = 135;
    public static double gateY = 59.25;

    public static double gateHeading = 41.25;

    public static double gateX1 = 133.5;
    public static double gateY1 = 58.75;
    public static double turretHeading1=60;
    public static double turretHeading2=55;
    public static double turretHeading3=75;
    public static double gateHeading1 = 42;
    private static final double MIN_ANGLE = -224.75;
    private static final double MAX_ANGLE =  224.75;
    private static final double TURRET_RANGE =  449.51;
    private double currentTurretPos = 180.0;

    private boolean matchStarted = false;
    private boolean autoShoot = false;
    private boolean useAutoGoalTracking = true;

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

    public void onInit() {
        autoShoot = false;
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
        turret1 = ActiveOpMode.hardwareMap().get(ServoImplEx.class, "turretServo1");
        turret2 = ActiveOpMode.hardwareMap().get(ServoImplEx.class,"turretServo2");
        turret1.setPwmRange(new PwmControl.PwmRange(500, 2500));
        turret2.setPwmRange(new PwmControl.PwmRange(500, 2500));
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
                enableGoalTracking(),
                new Delay(3),
                autoShootEnable(),
                new FollowPath(paths.MainChain)


        );
    }

    public void onStartButtonPressed() {
        opmodeTimer.resetTimer();
        matchStarted = true;

        Auto().schedule();
    }
    private boolean preload = false;
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
        Double[] results = calculateShotVectorandUpdateHeading(robotHeading, robotToGoalVector, follower.getVelocity().times(1), follower.getAcceleration());

        flywheelSpeed = results[0];

        if(preload==true){
            shooter(2500);
        }
        if(preload==false){
            shooter((float) flywheelSpeed - 38);
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
            // Default Vector Math Goal Tracking
            targetTurretAngle = getClosestValidTurretAngle(headingError + turretOffset - feedforwardOffset);
        }

        double servoPositionSignal = 0.05 + ((targetTurretAngle - MIN_ANGLE) / 449.51) * 0.90;
        servoPositionSignal = Math.max(0.05, Math.min(0.95, servoPositionSignal));
        turret1.setPosition(servoPositionSignal);
        turret2.setPosition(servoPositionSignal);
        currentTurretPos = targetTurretAngle;
        Pose futurepose = new Pose(follower.getPose().getX()+follower.getVelocity().getXComponent()*0.5, follower.getPose().getY()+follower.getVelocity().getYComponent()*0.3, follower.getHeading());

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

        public PathChain MainChain;

        public Paths(Follower follower) {
            MainChain = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(79.318, 9.2),
                                    new Pose(108.8133, 9.5)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(90))
                    .build();
        }
    }
}