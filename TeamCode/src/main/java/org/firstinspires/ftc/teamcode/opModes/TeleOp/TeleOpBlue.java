package org.firstinspires.ftc.teamcode.opModes.TeleOp;




import com.pedropathing.ivy.Command;
import org.firstinspires.ftc.teamcode.ivy.*;
import org.firstinspires.ftc.teamcode.pedroPathing.*;

import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.*;
import static org.firstinspires.ftc.teamcode.ivy.HardwareCommands.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.IvyPedroCommands.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.Poses.pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.TempHood;

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOpBlue")
public class TeleOpBlue extends IvyOpMode {

    public MotorEx intakeMotor;
    public MotorEx transfer;
    public TeleOpBlue() {
        configurePedro(Constants::create, TempHood.INSTANCE, DriveTrain.INSTANCE/*, Intake.INSTANCE, Spindexer.INSTANCE*/);
    }

    public static boolean blue;
    public static boolean isBlue(){
        return blue;
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




    private static final int APRILTAG_PIPELINE = 8;
    @Override
    public void onInit() {
//        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        limelight.pipelineSwitch(APRILTAG_PIPELINE);
//        limelight.start();
        blue=true;
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




    }

    @Override
    public void onUpdate() {
        float newtps=1000;
        /*if(lowerangle==true){
            newtps = findTPS44(DistanceBlue.INSTANCE.getDistanceFromTag());
            //RobotContext.telemetry().addData("Lowerangle:", lowerangle);
        }
        else if(lowerangle==false) {
            newtps = findTPS(DistanceBlue.INSTANCE.getDistanceFromTag());
            //RobotContext.telemetry().addData("Lowerangle:", lowerangle);
        }
        if (DistanceBlue.INSTANCE.getDistanceFromTag() != 0) {
            //shooter(newtps);
            RobotContext.telemetry().addData("newtps", newtps);
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
        blue=false;
    }
}