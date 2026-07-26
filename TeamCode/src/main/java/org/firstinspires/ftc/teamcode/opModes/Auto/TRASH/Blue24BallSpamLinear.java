package org.firstinspires.ftc.teamcode.opModes.Auto.TRASH;

import static org.firstinspires.ftc.teamcode.subsystems.DriveTrain2.closeStopperPos;
import static org.firstinspires.ftc.teamcode.subsystems.DriveTrain2.openStopperPos;
import static org.firstinspires.ftc.teamcode.subsystems.Flywheel.shooter;
import static org.firstinspires.ftc.teamcode.subsystems.LaunchDetector.isOverlappingLaunchZone;
import static org.firstinspires.ftc.teamcode.subsystems.ShooterCalc.calculateShotVectorandUpdateHeading;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Storage;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

@Disabled
@Autonomous
@Configurable
public class Blue24BallSpamLinear extends NextFTCOpMode {

    public Blue24BallSpamLinear() {
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

    // 144 - 109.810 = 34.190, Heading: 180 - 270 = -90
    public Pose start = new Pose(34.190, 132.272, Math.toRadians(270));

    // --- Turret tracking ---
    private ServoEx turret1;
    private ServoEx servoStopper;
    private ServoEx turret2;
    private ServoEx hoodServo;
    private MotorEx intakeMotor;
    double goalY = 144;
    double goalX = 0; // 144 - 144 = 0 (Goal is flipped to the blue side corner)

    // 144 - 132 = 12
    public static double gateX = 12;
    public static double gateY = 54.85;

    // Heading: 180 - 30 = 150
    public static double gateHeading = 150;

    // 144 - 131.4 = 12.6
    public static double gateX1 = 12.6;
    public static double gateY1 = 56;

    // Static turret targets transformed (180 - heading)
    public static double turretHeading1 = 120; // 180 - 60
    public static double turretHeading2 = 125; // 180 - 55
    public static double turretHeading3 = 105; // 180 - 75
    public static double gateHeading1 = 150;  // 180 - 30

    private static final double MIN_ANGLE = -224.75;
    private static final double MAX_ANGLE =  224.75;
    private static final double TURRET_RANGE =  449.51;
    private double currentTurretPos = 180.0;

    private boolean matchStarted = false;
    private boolean autoShoot = false;
    private boolean useAutoGoalTracking = true;

    private Command intakeMotorOn = new LambdaCommand()
            .setStart(() -> {
                intakeMotor.setPower(1);
                transfer.setPower(1);
            });

    public Command autoShootEnable(){
        return new LambdaCommand()
                .setStart(() -> autoShoot = true);
    }

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

    double targetTurretAngle;

    public Command setTurretHeading(double targetAngleDegrees) {
        return new LambdaCommand()
                .setStart(() -> {
                    useAutoGoalTracking = false;
                    targetTurretAngle = getClosestValidTurretAngle(targetAngleDegrees);
                });
    }

    public Command enableGoalTracking() {
        return new LambdaCommand()
                .setStart(() -> useAutoGoalTracking = true);
    }

    public double getClosestValidTurretAngle(double relativeGoalDegrees) {
        double option1 = relativeGoalDegrees;
        double option2 = (option1 > 180.0) ? (option1 - 360.0) : (option1 + 360.0);

        boolean opt1Valid = (option1 >= MIN_ANGLE && option1 <= MAX_ANGLE);
        boolean opt2Valid = (option2 >= MIN_ANGLE && option2 <= MAX_ANGLE);

        if (opt1Valid && opt2Valid) {
            return (Math.abs(option1 - currentTurretPos) < Math.abs(option2 - currentTurretPos)) ? option1 : option2;
        }

        if (opt1Valid) return option1;
        if (opt2Valid) return option2;

        return Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, option1));
    }

    public void onInit() {
        follower = PedroComponent.follower();
        follower.setStartingPose(start);
        paths = new Paths(follower);
        opmodeTimer = new Timer();
        intakeMotor = new MotorEx("intakeMotor");
        transfer = new MotorEx("transferMotor");
        turret1  = new ServoEx("turretServo1");
        turret2  = new ServoEx("turretServo2");
        hoodServo = new ServoEx("hoodServo");
        servoStopper = new ServoEx("stopperServo");
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

    public Command Auto() {
        return new SequentialGroup(
                setTurretHeading(20), // 180 - 160 = 20
                new FollowPath(paths.Path1, false, 1.0),

                intakeMotorOn,
                new FollowPath(paths.Path2, false, 1.0),
                setTurretHeading(turretHeading1),
                new FollowPath(paths.Path3, false, 1.0),

                setTurretHeading(turretHeading2),

                new FollowPath(paths.Path4, true, 1.0),
                new Delay(1.2),
                new FollowPath(paths.Path5, false, 1.0),

                new FollowPath(paths.Path6, true, 1.0),
                new Delay(1.8),

                new FollowPath(paths.Path7, false, 1.0),

                new FollowPath(paths.Path8, true, 1.0),
                new Delay(1.8),

                new FollowPath(paths.Path9, false, 1.0),

                new FollowPath(paths.Path10, true, 1.0),
                new Delay(1.8),
                new FollowPath(paths.Path11, false, 1.0),

                new FollowPath(paths.Path12, true, 1.0),
                new Delay(1.8),
                new FollowPath(paths.Path13, false, 1.0),

                setTurretHeading(turretHeading3),

                new FollowPath(paths.Path14, false, 1.0),
                intakeMotorOff,
                new FollowPath(paths.Path15, false, 1.0),

                new FollowPath(paths.Path16, false, 1.0)
        );
    }

    public void onStartButtonPressed() {
        opmodeTimer.resetTimer();
        matchStarted = true;
        Auto().schedule();
    }

    @Override
    public void onUpdate() {
        follower.update();
        Storage.currentPose = follower.getPose();

        if (!matchStarted) return;

        // --- Continuous Tracking / Calculation Logic ---
        Pose currPose = follower.getPose();
        double robotHeading = follower.getPose().getHeading();
        Vector robotToGoalVector = new Vector(follower.getPose().distanceFrom(new Pose(goalX, goalY)), Math.atan2(goalY - currPose.getY(), goalX - currPose.getX()));
        Double[] results = calculateShotVectorandUpdateHeading(robotHeading, robotToGoalVector, follower.getVelocity(), 1.3);
        double flywheelSpeed = results[0];
        shooter((float) flywheelSpeed);
        double hoodAngle = results[1];
        hoodServo.setPosition(hoodAngle);

        if (useAutoGoalTracking) {
            double headingError = results[2];
            targetTurretAngle = headingError;
        }

        // --- Hardware Execution Loop ---
        double servoPositionSignal = 0.05 + ((targetTurretAngle - MIN_ANGLE) / 449.51) * 0.90;
        servoPositionSignal = Math.max(0.05, Math.min(0.95, servoPositionSignal));
        turret1.setPosition(servoPositionSignal);
        turret2.setPosition(servoPositionSignal);

        currentTurretPos = ((turret1.getPosition() - 0.05) / 0.90) * 449.51 - 44.75;
        if(isOverlappingLaunchZone(PedroComponent.follower().getPose()) && robotToGoalVector.getMagnitude()>60&&autoShoot){
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
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;
        public PathChain Path10;
        public PathChain Path11;
        public PathChain Path12;
        public PathChain Path13;
        public PathChain Path14;
        public PathChain Path15;
        public PathChain Path16;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(34.190, 132.272),    // 144 - 109.810
                            new Pose(48.995, 94.650)))    // 144 - 95.005
                    .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-60)) // 180-270=-90, 180-240=-60
                    .addPoseCallback(new Pose(47.108, 99.572), autoShootEnable(), 0.9)      // 144 - 96.892
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(48.995, 94.650),     // 144 - 95.005
                            new Pose(59.236, 64),         // 144 - 84.764
                            new Pose(35.178, 61),         // 144 - 108.822
                            new Pose(18.210, 58.117)))    // 144 - 125.790
                    .setLinearHeadingInterpolation(Math.toRadians(-60), Math.toRadians(150)) // 180-240=-60, 180-30=150
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(18.210, 58.117),     // 144 - 125.790
                            new Pose(63.146, 69.703)))    // 144 - 80.854
                    .setLinearHeadingInterpolation(Math.toRadians(150), Math.toRadians(180)) // 180-30=150, 180-0=180
                    .build();

            Path4 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(63.146, 69.703),     // 144 - 80.854
                            new Pose(gateX1, gateY1)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(gateHeading1))
                    .build();

            Path5 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(gateX1, gateY1),
                            new Pose(61.942, 73.32)))     // 144 - 82.058
                    .setLinearHeadingInterpolation(Math.toRadians(gateHeading1), Math.toRadians(195)) // 180 - (-15) = 195
                    .build();

            Path6 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(61.942, 73.328),     // 144 - 82.058
                            new Pose(gateX - 0.05, gateY))) // 144 - (gateX + 0.05) calculated visually as dynamic offset shift
                    .setLinearHeadingInterpolation(Math.toRadians(195), Math.toRadians(gateHeading))
                    .build();

            Path7 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(gateX - 0.05, gateY),
                            new Pose(61.900, 73.328)))    // 144 - 82.100
                    .setLinearHeadingInterpolation(Math.toRadians(gateHeading), Math.toRadians(195))
                    .build();

            Path8 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(61.900, 73.328),
                            new Pose(gateX - 0.1, gateY)))
                    .setLinearHeadingInterpolation(Math.toRadians(195), Math.toRadians(gateHeading))
                    .build();

            Path9 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(gateX - 0.1, gateY),
                            new Pose(61.900, 73.328)))
                    .setLinearHeadingInterpolation(Math.toRadians(gateHeading), Math.toRadians(195))
                    .build();

            Path10 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(61.900, 73.328),
                            new Pose(gateX - 0.18, gateY)))
                    .setLinearHeadingInterpolation(Math.toRadians(195), Math.toRadians(gateHeading))
                    .build();

            Path11 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(gateX - 0.18, gateY),
                            new Pose(62.000, 73.328)))    // 144 - 82.000
                    .setLinearHeadingInterpolation(Math.toRadians(gateHeading), Math.toRadians(195))
                    .build();

            Path12 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(62.000, 73.328),
                            new Pose(gateX - 0.22, gateY)))
                    .setLinearHeadingInterpolation(Math.toRadians(195), Math.toRadians(gateHeading))
                    .build();

            Path13 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(gateX - 0.22, gateY),
                            new Pose(62, 73.328)))         // 144 - 82
                    .setLinearHeadingInterpolation(Math.toRadians(gateHeading), Math.toRadians(195))
                    .build();

            Path14 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(62.000, 73.328),     // 144 - 82
                            new Pose(56.191331496, 79.2375828716257), // 144 - 87.80866...
                            new Pose(24.203588545, 81.1828200643779))) // 144 - 119.79641...
                    .setLinearHeadingInterpolation(Math.toRadians(195), Math.toRadians(180)) // 180 - 0 = 180
                    .build();

            Path15 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(24.203588545, 81.1828200643779),
                            new Pose(49.033040376, 83.71111297536585))) // 144 - 94.96695...
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Path16 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(49.033, 83.711),     // 144 - 94.967
                            new Pose(39.223, 81.009)))    // 144 - 104.777
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();
        }
    }
}