package org.firstinspires.ftc.teamcode.subsystems;



import com.pedropathing.ivy.Command;
import org.firstinspires.ftc.teamcode.ivy.*;
import org.firstinspires.ftc.teamcode.pedroPathing.*;

import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.*;
import static org.firstinspires.ftc.teamcode.ivy.HardwareCommands.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.IvyPedroCommands.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.Poses.pose;
import static org.firstinspires.ftc.teamcode.opModes.TeleOp.TeleOpBlue.isBlue;
import static org.firstinspires.ftc.teamcode.opModes.TeleOp.TeleOpRed.isRed;
import static org.firstinspires.ftc.teamcode.subsystems.Flywheel.shooter;
import static org.firstinspires.ftc.teamcode.subsystems.LaunchDetector.isOverlappingLaunchZone;
import static org.firstinspires.ftc.teamcode.subsystems.ShooterCalc.calculateShotVectorandUpdateHeading;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.PolarVector;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.function.Supplier;

@Configurable
public class DriveTrain implements Subsystem {

    public static final DriveTrain INSTANCE = new DriveTrain();
    public DriveTrain() {
    }

    private Limelight3A limelight;

    private boolean hasTag;

    private double tx;


    public double aimMultiplier = 0.575;

    public static final MotorEx fL = new MotorEx("frontLeft").brakeMode();
    public static final MotorEx fR = new MotorEx("frontRight").brakeMode();
    public static final MotorEx bL = new MotorEx("backLeft").brakeMode();
    public static final MotorEx bR = new MotorEx("backRight").brakeMode();
    public static MotorEx flywheel2 = new MotorEx("launchingmotor2");
    public static MotorEx flywheel= new MotorEx("launchingmotor");

    private TeamFollower follower;

    public boolean firsttime = true;

    public int alliance;
    public boolean far;

    public Supplier<Double> yVCtx;

    public static double hoodToPos(double runtime) {
        if(Double.isNaN(runtime)!=true) {
            RobotContext.telemetry().addData("runtime", runtime);
            Command HoodRunUp = parallel(
                    position(hoodServo1, runtime),
                    position(hoodServo2, -1*runtime)
            );
            HoodRunUp.schedule();
            return runtime;
        }
        else {
            RobotContext.telemetry().addLine("NaN");
            return 0;
        }
    }



    //Pose startingpose = Storage.currentPose;
    Pose startingpose = pose(72,72, Math.toRadians(90));
    @Override
    public Command getDefaultCommand() {

        if (isBlue() != true && isRed() != true) {
            RobotContext.telemetry().addLine("No direction set");
        } else {
            if (isBlue() == true) {
                alliance = 1;
            }
            if (isRed() == true) {
                alliance = -1;
            }
        }
        return infinite(() -> follower.manual(
                alliance * IvyGamepads.gamepad1().leftStickY().getAsDouble(),
                alliance * IvyGamepads.gamepad1().leftStickX().getAsDouble(),
                0.75 * IvyGamepads.gamepad1().rightStickX().getAsDouble()
        )).requiring(this);
    }

    public Command localize;



    @Override
    public void initialize() {

        firsttime = true;
        shooting = false;
        follower = PedroRuntime.follower();
        if(isBlue()!=true && isRed()!=true) {
            RobotContext.telemetry().addLine("No direction set");
        }
        else{
            if(isBlue()==true) {
                alliance=1;
            }
            if(isRed()==true){
                alliance=-1;
            }}
        startingpose = Storage.currentPose;
        if(Storage.currentPose!=pose(0, 0, 0)) {
            follower.setStartingPose(startingpose);
        }

        if(alliance ==-1){
            localize = Command.build()
                    .setStart(()->follower.setPose(pose(129,90,Math.toRadians(90))));

        }
        if(alliance ==1){
            localize = Command.build()
                    .setStart(()->follower.setPose(pose(15,90,Math.toRadians(90))));

        }
        hoodServo1n= RobotContext.hardwareMap().get(Servo.class, "hoodServo1");
        hoodServo2n=  RobotContext.hardwareMap().get(Servo.class, "hoodServo2");
        follower.update();
    }


    private static MotorEx transfer1;
    private static ServoEx transfer2;

    double goalY = 138;
    double goalX = 138;

    static double localizeX;
    double goalXDist = 138;


    static boolean shooting = false;

    static Command shootFalse = Command.build()
            .setStart(() -> shooting=false);

    public boolean lift;

    public boolean decrease = false;

    static double transferpower = -1.0;

    public static Command opentransfer = Command.build()
            .setStart(()-> {
                //`5transfer2.setPosition(-0.25);
                transfer2.setPosition(0.35);
            }).setDone(() -> true);


    public static Command closeTransfer = Command.build()
            .setStart(() -> {
                transfer2.setPosition(0.635);
            }).setDone(() -> true);
    static Command transferOn = Command.build()
            .setStart(()-> transfer1.setPower(transferpower))
            .setDone(() -> true);
    static Command transferOff = Command.build()
            .setStart(() -> transfer1.setPower(0))
            .setDone(() -> true);


    public static void shoot(){
        if(shooting==false){
            shooting = true;
            Command shoot = sequential(opentransfer, waitMs((0.1) * 1000.0), transferOn, waitMs((0.4) * 1000.0), transferOff, closeTransfer, shootFalse);
            shoot.schedule();
        }
    }


    private static Servo hoodServo1n;
    private static Servo hoodServo2n;

    private static ServoEx hoodServo1 = new ServoEx(() -> hoodServo1n);
    private static ServoEx hoodServo2 = new ServoEx(() -> hoodServo2n);
    Command shooter = Command.build()
            .setStart(()-> shoot());
    public Command Localize(){
        return localize;
    }

    @Override
    public void periodic() {
        if (firsttime == true) {
            // Schedule the command stored in the localize variable
            IvyGamepads.gamepad1().x().whenBecomesTrue((()->Localize().schedule()));
            //IvyGamepads.gamepad1().square().whenBecomesTrue(() -> farAngle());
            IvyGamepads.gamepad1().rightTrigger().greaterThan(0.3).whenBecomesTrue(shooter);
            MotorEx intakeMotor = new MotorEx("intake");
            transfer1 = new MotorEx("transfer");
            transfer2 = new ServoEx("transferServo1");
            firsttime = false;
            Command HoodPowerZero=parallel(
                    position(hoodServo1, 0),
                    position(hoodServo2, 0)
            );
            HoodPowerZero.schedule();
        }
        follower.update();


        if (isBlue() == true) {
            goalXDist = 6;
            goalX = 6;
            localizeX = 136;
        }
        if (isRed() == true) {
            goalXDist = 138;
            goalX = 138;
            localizeX = 8;
        }
        Pose currPose = follower.getPose();
        double robotHeading = follower.getPose().heading();
        PolarVector robotToGoalVector = new PolarVector(follower.getPose().distance(pose(goalX, goalY)), Math.atan2(goalY - currPose.y(), goalX - currPose.x()));
        Double[] results = calculateShotVectorandUpdateHeading(robotHeading, robotToGoalVector, follower.getVelocity(), 1.3);
        Double headingError = results[2];
        double flywheelSpeed = results[0];
        shooter((float) flywheelSpeed);
        double hoodAngle = results[1];
        hoodToPos(hoodAngle);
        double s1speed = 60 * flywheel.getVelocity()/28;
        double s2speed = 60 * flywheel2.getVelocity()/28;
        if(isOverlappingLaunchZone(follower.getPose())){
            RobotContext.telemetry().addData("Launch?", isOverlappingLaunchZone(follower.getPose()));
        }
        if(isOverlappingLaunchZone(follower.getPose())){
            RobotContext.telemetry().addData("Launch in 0.2? TBD", isOverlappingLaunchZone(follower.getPose()));
        }

        //RobotContext.telemetry().addData("Motor1Speed", s1speed);
        //RobotContext.telemetry().addData("Motor2Speed", s2speed);
        RobotContext.telemetry().addData("far", far);
        RobotContext.telemetry().addData("alliance", alliance);
        //RobotContext.telemetry().addData("servo1pos", hoodServo1.getPosition());
        //RobotContext.telemetry().addData("servo2pos", hoodServo2.getPosition());

        //double frontLeftRPM = 28 / 60 * fL.getVelocity();
        //double frontRightRPM = 28 / 60 * fR.getVelocity();
        //double backLeftRPM = 28 / 60 * bL.getVelocity();
        //double backRightRPM = 28 / 60 * bR.getVelocity();
        //RobotContext.telemetry().addData("frontRightRPM", frontRightRPM);
        //RobotContext.telemetry().addData("backRightRPM", backRightRPM);
        //RobotContext.telemetry().addData("frontLeftRPM", frontLeftRPM);
        //RobotContext.telemetry().addData("backLeftRPM", backLeftRPM);

        //RobotContext.telemetry().addData("goalX", goalX);
        //RobotContext.telemetry().addData("goalY", goalY);
        RobotContext.telemetry().addData("RobotX", currPose.x());
        RobotContext.telemetry().addData("RobotY", currPose.y());
        //RobotContext.telemetry().addData("goalXDist", goalXDist);
        //RobotContext.telemetry().addData("goalYDist", goalYDist);
        //RobotContext.telemetry().addData("robotHeading", Math.toDegrees(robotHeading));
        //RobotContext.telemetry().addData("velocity", follower.getVelocity());
        RobotContext.telemetry().addData("headingError", headingError);
        //RobotContext.telemetry().addData("distance", distance);
        //RobotContext.telemetry().addData("yVCtx", visionYawCommand(headingError));
        RobotContext.telemetry().update();
    }
}
