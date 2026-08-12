package org.firstinspires.ftc.teamcode.opModes.TeleOp;



import org.firstinspires.ftc.teamcode.ivy.*;
import org.firstinspires.ftc.teamcode.pedroPathing.*;

import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.*;
import static org.firstinspires.ftc.teamcode.ivy.HardwareCommands.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.IvyPedroCommands.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.Poses.pose;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "servotest")
public class servotest extends IvyOpMode {
    public MotorEx intakeMotor;
    public MotorEx transfer;
    public servotest() {
        configurePedro(null);
    }

    private ServoImplEx turret1;
    private ServoImplEx turret2;

    @Override
    public void onInit() {
        turret1 = RobotContext.hardwareMap().get(ServoImplEx.class, "turretServo1");
        turret2 = RobotContext.hardwareMap().get(ServoImplEx.class,"turretServo1");
        turret1.setPwmRange(new PwmControl.PwmRange(500, 2500));
        turret2.setPwmRange(new PwmControl.PwmRange(500, 2500));
        telemetry.addLine("WARNING: Ensure White Turret Gear is Removed");

    }

    @Override
    public void onUpdate() {
        turret1.setPosition(0.5);
        turret2.setPosition(0.5);
    }


    @Override
    public void onStartButtonPressed() {

        turret1.setPosition(0.5);
        turret2.setPosition(0.5);
    }


    public void onStop(){

    }
}