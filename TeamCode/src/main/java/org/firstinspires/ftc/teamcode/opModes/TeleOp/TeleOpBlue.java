package org.firstinspires.ftc.teamcode.opModes.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.TempHood;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOpBlue")
public class TeleOpBlue extends NextFTCOpMode {

    public MotorEx intakeMotor;
    public MotorEx transfer;
    public TeleOpBlue() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(TempHood.INSTANCE, DriveTrain.INSTANCE/*, Intake.INSTANCE, Spindexer.INSTANCE*/),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE


        );
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
        Gamepads.gamepad1().leftTrigger().greaterThan(0.3).whenBecomesTrue(()-> intakeMotor.setPower(1))
                .whenBecomesFalse(() -> intakeMotor.setPower(0));
        Gamepads.gamepad1().leftBumper().whenBecomesTrue(()-> transfer.setPower(1))
                .whenBecomesFalse(() -> transfer.setPower(0));
        Gamepads.gamepad2().leftTrigger().greaterThan(0.3).whenBecomesTrue(()->intakeMotor.setPower(-1))
                .whenBecomesFalse(() -> intakeMotor.setPower(0));
        Gamepads.gamepad2().rightTrigger().greaterThan(0.3).whenBecomesTrue(()-> transfer.setPower(-1))
                .whenBecomesFalse(() -> intakeMotor.setPower(0));




    }

    @Override
    public void onUpdate() {
        float newtps=1000;
        /*if(lowerangle==true){
            newtps = findTPS44(DistanceBlue.INSTANCE.getDistanceFromTag());
            //ActiveOpMode.telemetry().addData("Lowerangle:", lowerangle);
        }
        else if(lowerangle==false) {
            newtps = findTPS(DistanceBlue.INSTANCE.getDistanceFromTag());
            //ActiveOpMode.telemetry().addData("Lowerangle:", lowerangle);
        }
        if (DistanceBlue.INSTANCE.getDistanceFromTag() != 0) {
            //shooter(newtps);
            ActiveOpMode.telemetry().addData("newtps", newtps);
        }*/
    }

    public boolean shoot;

    @Override
    public void onStartButtonPressed() {


        //Gamepads.gamepad2().cross().whenBecomesTrue(() -> hood());
        //Gamepads.gamepad2().triangle().whenBecomesTrue(() -> hoodMid());
        /*SequentialGroup onStart= new SequentialGroup(
                new Delay(2),
                //TempHood.INSTANCE.HoodUp,
                new SetPower(transfer, 0.25),
                new Delay(0.01),
                new SetPower(transfer, 0),
                TempHood.INSTANCE.HoodUp,
                new SetPower(transfer, 1),
                new Delay(0.5),
                TempHood.INSTANCE.HoodDown,
                new SetPower(transfer, 0)
        );
        //int tag=MotifScanning.INSTANCE.findMotif();
        onStart.schedule();*/
    }


    public void onStop(){
        blue=false;
    }
}