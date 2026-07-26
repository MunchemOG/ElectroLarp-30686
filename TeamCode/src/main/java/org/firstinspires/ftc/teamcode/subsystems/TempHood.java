package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;

import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.powerable.SetPower;


@Configurable
public class TempHood implements Subsystem {



    public static final TempHood INSTANCE = new TempHood();
    private TempHood() { }

    private static CRServo hoodServo1n;
    private static CRServo hoodServo2n;

    private static CRServoEx hoodServo1 = new CRServoEx(() -> hoodServo1n);
    private static CRServoEx hoodServo2 = new CRServoEx(() -> hoodServo2n);




    public static ParallelGroup HoodRunUp=new ParallelGroup(
            new SetPower(hoodServo1,-1),
            new SetPower(hoodServo2,1)
    );

    public static ParallelGroup HoodPowerZero=new ParallelGroup(
            new SetPower(hoodServo1,0),
            new SetPower(hoodServo2,0)
    );

    public static SequentialGroup HoodUp=new SequentialGroup(
            HoodRunUp,
            new Delay(0.18),
            HoodPowerZero
    );

    /*public SequentialGroup HoodUpMidRange=new SequentialGroup(
            HoodRunUp,
            new Delay(0.05),
            HoodPowerZero
    );*/

    public static ParallelGroup HoodRunDown=new ParallelGroup(
            new SetPower(hoodServo1,1),
            new SetPower(hoodServo2,-1)
    );

    public static SequentialGroup HoodDown=new SequentialGroup(
            HoodRunDown,
            new Delay(0.17),
            HoodPowerZero
    );

    public static double hoodUp(double runtime, double currentstate) {
        if(Double.isNaN(runtime)!=true){
        ActiveOpMode.telemetry().addData("runtime", runtime);
        ActiveOpMode.telemetry().addData("currentstate", currentstate);
        SequentialGroup runUp = new SequentialGroup(
                HoodRunUp,
                HoodRunUp,
                HoodRunUp,
                HoodRunUp,
                new Delay(runtime - currentstate),
                HoodPowerZero
        );
        SequentialGroup runDown = new SequentialGroup(
                HoodRunDown,
                new Delay(currentstate - runtime),
                HoodPowerZero
        );
        if(runtime>currentstate+0.007) {
            runUp.schedule();
            ActiveOpMode.telemetry().addLine("runUp");
            return runtime;
        }
        if(runtime<currentstate-0.007){
            runDown.schedule();
            ActiveOpMode.telemetry().addLine("runDown");
            return runtime;
        }
        else{
            ActiveOpMode.telemetry().addLine("returning0");
            return 0;
        }}
        else{
            ActiveOpMode.telemetry().addLine("NaN");
            return 0;
        }
    }




    @Override
    public void initialize(){
        //hoodServo1n= ActiveOpMode.hardwareMap().get(CRServo.class, "hoodServo1");
        //hoodServo2n=  ActiveOpMode.hardwareMap().get(CRServo.class, "hoodServo2");


    }

    @Override
    public void periodic() {

    }
}
