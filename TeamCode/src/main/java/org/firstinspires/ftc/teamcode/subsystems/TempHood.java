package org.firstinspires.ftc.teamcode.subsystems;



import com.pedropathing.ivy.Command;
import org.firstinspires.ftc.teamcode.ivy.*;
import org.firstinspires.ftc.teamcode.pedroPathing.*;

import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.*;
import static org.firstinspires.ftc.teamcode.ivy.HardwareCommands.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.IvyPedroCommands.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.Poses.pose;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;

@Configurable
public class TempHood implements Subsystem {



    public static final TempHood INSTANCE = new TempHood();
    private TempHood() { }

    private static CRServo hoodServo1n;
    private static CRServo hoodServo2n;

    private static CRServoEx hoodServo1 = new CRServoEx(() -> hoodServo1n);
    private static CRServoEx hoodServo2 = new CRServoEx(() -> hoodServo2n);




    public static Command HoodRunUp=parallel(
            power(hoodServo1, -1),
            power(hoodServo2, 1)
    );

    public static Command HoodPowerZero=parallel(
            power(hoodServo1, 0),
            power(hoodServo2, 0)
    );

    public static Command HoodUp=sequential(
            HoodRunUp,
            waitMs((0.18) * 1000.0),
            HoodPowerZero
    );

    /*public Command HoodUpMidRange=sequential(
            HoodRunUp,
            waitMs((0.05) * 1000.0),
            HoodPowerZero
    );*/

    public static Command HoodRunDown=parallel(
            power(hoodServo1, 1),
            power(hoodServo2, -1)
    );

    public static Command HoodDown=sequential(
            HoodRunDown,
            waitMs((0.17) * 1000.0),
            HoodPowerZero
    );

    public static double hoodUp(double runtime, double currentstate) {
        if(Double.isNaN(runtime)!=true){
        RobotContext.telemetry().addData("runtime", runtime);
        RobotContext.telemetry().addData("currentstate", currentstate);
        Command runUp = sequential(
                HoodRunUp,
                HoodRunUp,
                HoodRunUp,
                HoodRunUp,
                waitMs((runtime - currentstate) * 1000.0),
                HoodPowerZero
        );
        Command runDown = sequential(
                HoodRunDown,
                waitMs((currentstate - runtime) * 1000.0),
                HoodPowerZero
        );
        if(runtime>currentstate+0.007) {
            runUp.schedule();
            RobotContext.telemetry().addLine("runUp");
            return runtime;
        }
        if(runtime<currentstate-0.007){
            runDown.schedule();
            RobotContext.telemetry().addLine("runDown");
            return runtime;
        }
        else{
            RobotContext.telemetry().addLine("returning0");
            return 0;
        }}
        else{
            RobotContext.telemetry().addLine("NaN");
            return 0;
        }
    }




    @Override
    public void initialize(){
        //hoodServo1n= RobotContext.hardwareMap().get(CRServo.class, "hoodServo1");
        //hoodServo2n=  RobotContext.hardwareMap().get(CRServo.class, "hoodServo2");


    }

    @Override
    public void periodic() {

    }
}
