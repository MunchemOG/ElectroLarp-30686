
package org.firstinspires.ftc.teamcode.opModes.Auto.TRASH;

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
import org.firstinspires.ftc.teamcode.subsystems.AutoShooterCalc;
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
public class Red21BallSpamLinear extends NextFTCOpMode {

    public Red21BallSpamLinear() {
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

    public Pose start = new Pose(startX, startY, Math.toRadians(270));

    // --- Turret tracking ---
    private ServoEx servoStopper;
    private ServoEx hoodServo;
    public static double startX = 110.17;
    public static double startY = 134.4;

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

    public static double turretOffset = -10;
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
                servoStopper.setPosition(closeStopperPos);
            }).setIsDone(() -> true);
    public Command openStopper = new LambdaCommand()
            .setStart(() -> {
                servoStopper.setPosition(openStopperPos);
            }).setIsDone(() -> true);


    public Command Auto(){
        return new SequentialGroup(
                setTurretHeading(0),
                new FollowPath(paths.preloadLaunch,true,1.0),
                shoot,
                intakeMotorOn,
                servoClose,
                turnOffPreload(),
                new FollowPath(paths.intakeSet2,true,1.0),



                new FollowPath(paths.launchSet2,true,1.0),
                shoot,
                intakeMotorOn,


                new FollowPath(paths.resetAndIntake1,true,1.0),
                new Delay(1.25),
                //reverseIntakeForMe,
                new FollowPath(paths.launchSpam1, true, 1.0),
                shoot,

                intakeMotorOn,

                new FollowPath(paths.resetAndIntake2, true, 1.0),
                new Delay(2.2),
                //reverseIntakeForMe,
                new FollowPath(paths.launchSpam2, true, 1.0),
                shoot,
                intakeMotorOn,
                new FollowPath(paths.resetAndIntake2, true, 1.0),
                new Delay(2.2),
                //reverseIntakeForMe,
                new FollowPath(paths.launchSpam2, true, 1.0),
                shoot,
                intakeMotorOn,
                new FollowPath(paths.intakeSet1,true,1.0),
                new FollowPath(paths.launchSet1,true,1.0),
                shoot,
                intakeMotorOn,
                new FollowPath(paths.intakeSet3,true,1.0),
                new FollowPath(paths.launchSet3,true,1.0),
                new FollowPath(paths.teleOpPark,true,1.0)
        );
    }

    public void onStartButtonPressed() {
        opmodeTimer.resetTimer();
        matchStarted = true;
        shooter(2100);
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
        Double[] results = AutoShooterCalc.calculateShotVectorandUpdateHeading(robotHeading, robotToGoalVector, follower.getVelocity().times(1), follower.getAcceleration());

        flywheelSpeed = results[0];

        if(preload==true){
            shooter(2100);
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
    public SequentialGroup shoot = new SequentialGroup(
            openStopper,
            intakeMotorOn,
            new Delay(0.3),
            closeStopper,
            intakeMotorOff
    );
    public Command reverseIntakeForMe = new LambdaCommand()
            .setStart(() -> intakeMotor.setPower(-1));
    public class Paths {
        public PathChain preloadLaunch;
        public PathChain intakeSet2;
        public PathChain launchSet2;
        public PathChain resetAndIntake1;
        public PathChain moverBacker;
        public PathChain launchSpam1;
        public PathChain resetAndIntake2;
        public PathChain launchSpam2;
        public PathChain intakeSet1;
        public PathChain launchSet1;
        public PathChain intakeSet3;
        public PathChain launchSet3;
        public PathChain teleOpPark;

        public Paths(Follower follower) {
            preloadLaunch = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(112.158,135.289),
                                    new Pose(110.000, 110.500),
                                    new Pose(100.000, 100.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90),Math.toRadians(46))
                    .addPoseCallback(new Pose(102,105),autoShootEnable(),0.8)
                    .setVelocityConstraint(1.0)
                    .setTValueConstraint(0.8)

                    .build();

            intakeSet2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(100.000, 100.000),
                                    new Pose(85, 56.798),
                                    new Pose(130, 60.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(46), Math.toRadians(-15))
                    .setVelocityConstraint(1.0)
                    .setTValueConstraint(0.8)



                    .build();

            launchSet2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(130, 60.000),
                                    new Pose(104.000, 67.000),
                                    new Pose(92.000, 94.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(47))
                    .setVelocityConstraint(0.3)
                    .setTValueConstraint(0.95)
                    //.addTemporalCallback(0.7,reverseIntakeForMe)
                    //.addTemporalCallback(0.2,transferOff)

                    .build();
//public static double gateX = 135;
//    public static double gateY = 59.25;
            resetAndIntake1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(92.000, 94.000),
                                    new Pose(104.000, 67.000),
                                    new Pose(135, 59.65)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(47), Math.toRadians(0))
                    .setVelocityConstraint(1.0)
                    .setTValueConstraint(0.8)
                    .addTemporalCallback(0.1,intakeMotorOn)


                    .build();

           /* moverBacker = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(128, 64),
                                    new Pose(130, 59.75)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(20), Math.toRadians(45))
                    .setVelocityConstraint(1.0)
                    .setTValueConstraint(0.8)

                    .build();*/

            launchSpam1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(135, 57.65),
                                    new Pose(104.000, 67.000),
                                    new Pose(92.000, 94.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(46))
                    .setVelocityConstraint(0.3)
                    .setTValueConstraint(0.95)
                    //.addTemporalCallback(0.9,reverseIntakeForMe)
                    .addPoseCallback(new Pose(124,61),reverseIntakeForMe,0.3)

                    .build();

            resetAndIntake2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(92.000, 94.000),
                                    new Pose(100, 67.000),
                                    new Pose(135, 59.25)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(46), Math.toRadians(40))
                    .setVelocityConstraint(1.0)
                    .setTValueConstraint(0.8)
                    .addTemporalCallback(0.1,intakeMotorOn)

                    .build();

            launchSpam2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(135, 59.25),
                                    new Pose(104.000, 67.000),
                                    new Pose(92, 94)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(17), Math.toRadians(47))
                    .setVelocityConstraint(0.3)
                    .setTValueConstraint(0.95)
                    .addPoseCallback(new Pose(118,64),reverseIntakeForMe,0.4)


                    //.addTemporalCallback(0.9,reverseIntakeForMe)
                    // .addTemporalCallback(0.7,transferOff)


                    .build();

            intakeSet1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(92, 94),
                                    new Pose(104,87),

                                    new Pose(126.665, 86.898)
                            )
                    ).setTangentHeadingInterpolation()
                    .setVelocityConstraint(1.0)
                    .setTValueConstraint(0.8)
                    .addTemporalCallback(0.1,intakeMotorOn)


                    .build();

            launchSet1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(126.665, 86.898),

                                    new Pose(92.000, 94.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(1), Math.toRadians(53))
                    .setVelocityConstraint(0.3)
                    .setTValueConstraint(0.95)
                    .addPoseCallback(new Pose(102,86),reverseIntakeForMe,0.4)

                    //.addTemporalCallback(0.7,reverseIntakeForMe)
                    //.addTemporalCallback(0.7,transferOff)

                    .build();

            intakeSet3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(92.000, 94.000),
                                    new Pose(88.000, 89.500),
                                    new Pose(75, 28.615),
                                    new Pose(129.151, 38)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(53), Math.toRadians(0))
                    .setVelocityConstraint(1.0)
                    .setTValueConstraint(0.8)
                    .addTemporalCallback(0.1,intakeMotorOn)


                    .build();

            launchSet3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(129.151, 36.938),

                                    new Pose(84, 103)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(1), Math.toRadians(43))

                    .setVelocityConstraint(0.3)
                    .setTValueConstraint(0.8)
                    .addPoseCallback(new Pose(107,69),reverseIntakeForMe,0.7)
                    .addPoseCallback(new Pose(94,88),shoot,0.8)

                    //.addTemporalCallback(0.7,reverseIntakeForMe)
                    //.addTemporalCallback(0.7,transferOff)

                    .build();

            teleOpPark = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(84, 103),

                                    new Pose(91, 115)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(30),Math.toRadians(90))

                    .build();
        }
    }
}