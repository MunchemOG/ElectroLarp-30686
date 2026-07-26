package org.firstinspires.ftc.teamcode.opModes.TeleOp;


import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;



@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "servotest")
public class servotest extends NextFTCOpMode {
    public MotorEx intakeMotor;
    public MotorEx transfer;
    public servotest() {
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE


        );
    }

    private ServoImplEx turret1;
    private ServoImplEx turret2;

    @Override
    public void onInit() {
        turret1 = ActiveOpMode.hardwareMap().get(ServoImplEx.class, "turretServo1");
        turret2 = ActiveOpMode.hardwareMap().get(ServoImplEx.class,"turretServo1");
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