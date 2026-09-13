package org.firstinspires.ftc.teamcode.subsystems;



import com.pedropathing.ivy.Command;
import org.firstinspires.ftc.teamcode.ivy.*;
import org.firstinspires.ftc.teamcode.pedroPathing.*;

import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.*;
import static org.firstinspires.ftc.teamcode.ivy.HardwareCommands.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.IvyPedroCommands.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.Poses.pose;
import com.qualcomm.robotcore.hardware.CRServo;

public class Airsort implements Subsystem {
    public static final Airsort INSTANCE = new Airsort();
    private Airsort() { }
    private CRServo hoodServo1n;
    private CRServo hoodServo2n;

    private CRServoEx hoodServo1 = new CRServoEx(() -> hoodServo1n);
    private CRServoEx hoodServo2 = new CRServoEx(() -> hoodServo2n);


    public static MotorEx transfer;



    Command HoodRunUp=parallel(
            power(hoodServo1, 1),
            power(hoodServo2, -1)
    );

    public Command HoodPowerZero=parallel(
            power(hoodServo1, 0),
            power(hoodServo2, 0)
    );

    public Command HoodDown=sequential(
            HoodRunUp,
            waitMs((0.18) * 1000.0),
            HoodPowerZero
    );

    Command HoodRunDown=parallel(
            power(hoodServo1, -1),
            power(hoodServo2, 1)
    );

    public Command HoodUp=sequential(
            HoodRunDown,
            waitMs((0.17) * 1000.0),
            HoodPowerZero
    );

    Command PPGtoPGP /*SAME AS PGP TO PPG*/= sequential(
            power(transfer, 0.25),
            TempHood.INSTANCE.HoodUp, //USED AS A DELAY HERE
            power(transfer, 0),
            waitMs((0.18) * 1000.0),
            power(transfer, 0.25),
            TempHood.INSTANCE.HoodUp,
            power(transfer, 0),
            TempHood.INSTANCE.HoodUp,
            power(transfer, 0.4),
            waitMs((0.2) * 1000.0),
            TempHood.INSTANCE.HoodDown,
            power(transfer, 0)
    );
    Command GPPtoPGP /*SAME AS PGPtoGPP*/= sequential(
            power(transfer, 0.25),
            TempHood.INSTANCE.HoodUp,
            power(transfer, 0),
            TempHood.INSTANCE.HoodUp,
            power(transfer, 0.25),
            waitMs((0.05) * 1000.0),
            power(transfer, 0),
            TempHood.INSTANCE.HoodDown,
            waitMs((0.18) * 1000.0),
            power(transfer, 1),
            waitMs((0.2) * 1000.0),
            power(transfer, 0)
    );
    Command PGPtoPPG /*SAME AS PPG TO PGP*/ = sequential(
            power(transfer, 0.25),
            TempHood.INSTANCE.HoodUp, //USED AS A DELAY HERE
            power(transfer, 0),
            waitMs((0.18) * 1000.0),
            power(transfer, 0.25),
            TempHood.INSTANCE.HoodUp,
            power(transfer, 0),
            TempHood.INSTANCE.HoodUp,
            power(transfer, 0.4),
            waitMs((0.2) * 1000.0),
            TempHood.INSTANCE.HoodDown,
            power(transfer, 0)
    );
//    Command GPPtoPPG = sequential(
//            power(transfer, 0.25),
//            TempHood.INSTANCE.HoodUp,
//            power(transfer, 0),
//            TempHood.INSTANCE.HoodUp,
//            power(transfer, 1),
//            waitMs((0.35) * 1000.0),
//            TempHood.INSTANCE.HoodDown,
//            power(transfer, 0)
//    );
//    Command PPGtoGPP = sequential(
//            power(transfer, 1),
//            TempHood.INSTANCE.HoodUp,
//            TempHood.INSTANCE.HoodUp,
//            waitMs((0.35) * 1000.0),
//            TempHood.INSTANCE.HoodDown,
//            power(transfer, 0)
//    );
//    Command PGPtoGPP /*SAME AS GPPtoPGP*/ = sequential(
//            power(transfer, 0 .25),
//            TempHood.INSTANCE.HoodUp,
//            power(transfer, 0),
//            TempHood.INSTANCE.HoodUp,
//            power(transfer, 0.25),
//            waitMs((0.05) * 1000.0),
//            power(transfer, 0),
//            TempHood.INSTANCE.HoodDown,
//            waitMs((0.18) * 1000.0),
//            power(transfer, 1),
//            waitMs((0.2) * 1000.0),
//            power(transfer, 0)
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
//            .withDefault(power(transfer, 1));


    @Override
    public void initialize() {

        hoodServo1n= RobotContext.hardwareMap().get(CRServo.class, "hoodServo1");
        hoodServo2n=  RobotContext.hardwareMap().get(CRServo.class, "hoodServo2");
        transfer = new MotorEx("transfer").reversed();


    }

    @Override
    public void periodic() {

    }
}
