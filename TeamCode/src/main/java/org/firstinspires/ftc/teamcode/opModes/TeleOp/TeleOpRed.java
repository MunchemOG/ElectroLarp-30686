package org.firstinspires.ftc.teamcode.opModes.TeleOp;




import com.pedropathing.ivy.Command;
import org.firstinspires.ftc.teamcode.ivy.*;
import org.firstinspires.ftc.teamcode.pedroPathing.*;

import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.*;
import static org.firstinspires.ftc.teamcode.ivy.HardwareCommands.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.IvyPedroCommands.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.Poses.pose;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.TempHood;

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOpRed")
public class TeleOpRed extends IvyOpMode {

    public MotorEx intakeMotor;
    public MotorEx transfer;
    public TeleOpRed() {
        configurePedro(Constants::create, TempHood.INSTANCE, DriveTrain.INSTANCE/*, Intake.INSTANCE, Spindexer.INSTANCE*/);
    }

    public static boolean red;
    public static boolean isRed(){
        return red;
    }

    public static int tagID;
    public static boolean findMotif = false;
    public static int ball1Color = 0; //green = 1, purple = 2
    public static int ball2Color = 0;
    public static int ball3Color = 0;

    public static int getBall1Color() {
        return ball1Color;
    }

    public static int getBall2Color() {
        return ball2Color;
    }
    public static int getBall3Color() {
        return ball3Color;
    }
    public boolean lift;
    boolean lowerangle = false;





    public boolean liftmid;
    boolean loweranglemid = false;

    public boolean isInLaunchZone(double x, double y) {

        // Triangle 1: Goal Side (Top)
        // Vertices: (-8, 144), (152, 144), (72, 64)
        // This triangle exists between y = 64 and y = 144.
        if (y >= 64 && y <= 144) {
            // As y increases from 64 to 144, the width of the triangle increases.
            // The slope of the edges is (144 - 64) / (152 - 72) = 80 / 80 = 1.
            double halfWidth = (y - 64);
            if (x >= (72 - halfWidth) && x <= (72 + halfWidth)) {
                return true;
            }
        }

        // Triangle 2: Audience Side (Bottom)
        // Vertices: (72, 32), (104, 0), (40, 0)
        // This triangle exists between y = 0 and y = 32.
        if (y >= 0 && y <= 32) {
            // As y decreases from 32 to 0, the width increases.
            // The slope of the edges is (32 - 0) / (72 - 40) = 32 / 32 = 1.
            double halfWidth = (32 - y);
            if (x >= (72 - halfWidth) && x <= (72 + halfWidth)) {
                return true;
            }
        }

        return false;
    }




    private static final int APRILTAG_PIPELINE = 7;
    @Override
    public void onInit() {
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        red=true;
        intakeMotor = new MotorEx("intake").reversed();
        transfer = new MotorEx("transfer").reversed();
        IvyGamepads.gamepad1().leftTrigger().greaterThan(0.3).whenBecomesTrue(()-> intakeMotor.setPower(1))
                .whenBecomesFalse(() -> intakeMotor.setPower(0));
        IvyGamepads.gamepad1().leftBumper().whenBecomesTrue(()-> transfer.setPower(1))
                .whenBecomesFalse(() -> transfer.setPower(0));
        IvyGamepads.gamepad2().leftTrigger().greaterThan(0.3).whenBecomesTrue(()->intakeMotor.setPower(-1))
                .whenBecomesFalse(() -> intakeMotor.setPower(0));
        IvyGamepads.gamepad2().rightTrigger().greaterThan(0.3).whenBecomesTrue(()-> transfer.setPower(-1))
                .whenBecomesFalse(() -> intakeMotor.setPower(0));
        IvyGamepads.gamepad1().rightBumper().whenBecomesTrue(() -> DriveTrain.opentransfer.schedule())
                .whenBecomesFalse(() -> DriveTrain.closeTransfer.schedule());
        IvyGamepads.gamepad1().x().whenBecomesTrue(()->PedroRuntime.follower().setPose(pose(79.967,9.271,Math.toRadians(90))));







    }

    @Override
    public void onUpdate() {
        /*float newtps=1000;
        if(lowerangle==true){
            newtps = findTPS44(DistanceRed.INSTANCE.getDistanceFromTag());
            //RobotContext.telemetry().addData("Lowerangle:", lowerangle);
        }
        else if(lowerangle==false) {
            newtps = findTPS(DistanceRed.INSTANCE.getDistanceFromTag());
            //RobotContext.telemetry().addData("Lowerangle:", lowerangle);
        }
        if (DistanceRed.INSTANCE.getDistanceFromTag() != 0) {
            shooter(newtps);
            //RobotContext.telemetry().addData("newtps", newtps);
        }*/
    }

    public boolean shoot;
 
    @Override
    public void onStartButtonPressed() {


        //IvyGamepads.gamepad2().cross().whenBecomesTrue(() -> hood());
        //IvyGamepads.gamepad2().triangle().whenBecomesTrue(() -> hoodMid());
        /*Command onStart= sequential(
                waitMs((2) * 1000.0),
                //TempHood.INSTANCE.HoodUp,
                power(transfer, 0.25),
                waitMs((0.01) * 1000.0),
                power(transfer, 0),
                TempHood.INSTANCE.HoodUp,
                power(transfer, 1),
                waitMs((0.5) * 1000.0),
                TempHood.INSTANCE.HoodDown,
                power(transfer, 0)
        );
        //int tag=MotifScanning.INSTANCE.findMotif();
        onStart.schedule();*/
    }


    public void onStop(){
        red=false;
    }
}
