package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;

import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class Airsort implements Subsystem {
    public static final Airsort INSTANCE = new Airsort();
    private Airsort() { }
    private CRServo hoodServo1n;
    private CRServo hoodServo2n;

    private CRServoEx hoodServo1 = new CRServoEx(() -> hoodServo1n);
    private CRServoEx hoodServo2 = new CRServoEx(() -> hoodServo2n);


    public static MotorEx transfer;



    ParallelGroup HoodRunUp=new ParallelGroup(
            new SetPower(hoodServo1,1),
            new SetPower(hoodServo2,-1)
    );

    public ParallelGroup HoodPowerZero=new ParallelGroup(
            new SetPower(hoodServo1,0),
            new SetPower(hoodServo2,0)
    );

    public SequentialGroup HoodDown=new SequentialGroup(
            HoodRunUp,
            new Delay(0.18),
            HoodPowerZero
    );

    ParallelGroup HoodRunDown=new ParallelGroup(
            new SetPower(hoodServo1,-1),
            new SetPower(hoodServo2,1)
    );

    public SequentialGroup HoodUp=new SequentialGroup(
            HoodRunDown,
            new Delay(0.17),
            HoodPowerZero
    );

    SequentialGroup PPGtoPGP /*SAME AS PGP TO PPG*/= new SequentialGroup(
            new SetPower(transfer, 0.25),
            TempHood.INSTANCE.HoodUp, //USED AS A DELAY HERE
            new SetPower(transfer, 0),
            new Delay(0.18),
            new SetPower(transfer, 0.25),
            TempHood.INSTANCE.HoodUp,
            new SetPower(transfer, 0),
            TempHood.INSTANCE.HoodUp,
            new SetPower(transfer, 0.4),
            new Delay(0.2),
            TempHood.INSTANCE.HoodDown,
            new SetPower(transfer, 0)
    );
    SequentialGroup GPPtoPGP /*SAME AS PGPtoGPP*/= new SequentialGroup(
            new SetPower(transfer, 0.25),
            TempHood.INSTANCE.HoodUp,
            new SetPower(transfer, 0),
            TempHood.INSTANCE.HoodUp,
            new SetPower(transfer, 0.25),
            new Delay(0.05),
            new SetPower(transfer, 0),
            TempHood.INSTANCE.HoodDown,
            new Delay(0.18),
            new SetPower(transfer, 1),
            new Delay(0.2),
            new SetPower(transfer, 0)
    );
    SequentialGroup PGPtoPPG /*SAME AS PPG TO PGP*/ = new SequentialGroup(
            new SetPower(transfer, 0.25),
            TempHood.INSTANCE.HoodUp, //USED AS A DELAY HERE
            new SetPower(transfer, 0),
            new Delay(0.18),
            new SetPower(transfer, 0.25),
            TempHood.INSTANCE.HoodUp,
            new SetPower(transfer, 0),
            TempHood.INSTANCE.HoodUp,
            new SetPower(transfer, 0.4),
            new Delay(0.2),
            TempHood.INSTANCE.HoodDown,
            new SetPower(transfer, 0)
    );
//    SequentialGroup GPPtoPPG = new SequentialGroup(
//            new SetPower(transfer, 0.25),
//            TempHood.INSTANCE.HoodUp,
//            new SetPower(transfer, 0),
//            TempHood.INSTANCE.HoodUp,
//            new SetPower(transfer, 1),
//            new Delay(0.35),
//            TempHood.INSTANCE.HoodDown,
//            new SetPower(transfer, 0)
//    );
//    SequentialGroup PPGtoGPP = new SequentialGroup(
//            new SetPower(transfer, 1),
//            TempHood.INSTANCE.HoodUp,
//            TempHood.INSTANCE.HoodUp,
//            new Delay(0.35),
//            TempHood.INSTANCE.HoodDown,
//            new SetPower(transfer, 0)
//    );
//    SequentialGroup PGPtoGPP /*SAME AS GPPtoPGP*/ = new SequentialGroup(
//            new SetPower(transfer, 0 .25),
//            TempHood.INSTANCE.HoodUp,
//            new SetPower(transfer, 0),
//            TempHood.INSTANCE.HoodUp,
//            new SetPower(transfer, 0.25),
//            new Delay(0.05),
//            new SetPower(transfer, 0),
//            TempHood.INSTANCE.HoodDown,
//            new Delay(0.18),
//            new SetPower(transfer, 1),
//            new Delay(0.2),
//            new SetPower(transfer, 0)
//    );
//
//
//    SwitchCommand autoAirsort = new SwitchCommand(() -> "text")
//            .withCase("PGP1", PPGtoPGP)
//            .withCase("PGP3", GPPtoPGP)
//            .withCase("PPG2", PGPtoPPG)
//            .withCase("PPG3", GPPtoPPG)
//            .withCase("GPP1", PPGtoGPP)
//            .withCase("GPP2", PGPtoGPP)
//            .withDefault(new SetPower(transfer, 1));


    @Override
    public void initialize() {

        hoodServo1n= ActiveOpMode.hardwareMap().get(CRServo.class, "hoodServo1");
        hoodServo2n=  ActiveOpMode.hardwareMap().get(CRServo.class, "hoodServo2");
        transfer = new MotorEx("transfer").reversed();


    }

    @Override
    public void periodic() {

    }
}
