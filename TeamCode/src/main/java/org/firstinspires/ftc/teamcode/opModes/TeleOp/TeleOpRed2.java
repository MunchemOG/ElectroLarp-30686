package org.firstinspires.ftc.teamcode.opModes.TeleOp;


import org.firstinspires.ftc.teamcode.ivy.*;
import org.firstinspires.ftc.teamcode.pedroPathing.*;

import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.*;
import static org.firstinspires.ftc.teamcode.ivy.HardwareCommands.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.IvyPedroCommands.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.Poses.pose;
import static org.firstinspires.ftc.teamcode.subsystems.DriveTrain2.intakeMotor;
import static org.firstinspires.ftc.teamcode.subsystems.DriveTrain2.transfer;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain2;
import org.firstinspires.ftc.teamcode.subsystems.ShooterCalc;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOpRed")
public class TeleOpRed2 extends IvyOpMode {
    //public MotorEx intakeMotor;
    //public MotorEx transfer;

    public TeleOpRed2() {
        configurePedro(Constants::create, DriveTrain2.INSTANCE);
    }

    public static boolean red;
    public static boolean isRed(){
        return red;
    }

    @Override
    public void onInit() {
        red=true;
        //intakeMotor = new MotorEx("intakeMotor");
        //transfer = new MotorEx("transferMotor");
        // OPTIMIZATION: Single trigger binding for both motors
        IvyGamepads.gamepad1().leftTrigger().greaterThan(0.3)
                .whenBecomesTrue(() -> {
                    intakeMotor.setPower(1);
                    transfer.setPower(1);
                })
                .whenBecomesFalse(() -> {
                    intakeMotor.setPower(0);
                    transfer.setPower(0);
                });


        // Backup Controls
        // These only fire on edge transitions, so loop impact is minimal
        IvyGamepads.gamepad2().leftTrigger().greaterThan(0.5).whenBecomesTrue(() -> DriveTrain2.turretOffset -= DriveTrain2.turretOffsetStep);
        IvyGamepads.gamepad2().rightTrigger().greaterThan(0.5).whenBecomesTrue(() -> DriveTrain2.turretOffset += DriveTrain2.turretOffsetStep);
        IvyGamepads.gamepad2().rightBumper().whenBecomesTrue(()->DriveTrain2.turretOffset+= 1);
        IvyGamepads.gamepad2().leftBumper().whenBecomesTrue(()->DriveTrain2.turretOffset-= 1);
        IvyGamepads.gamepad2().a().whenBecomesTrue(() -> DriveTrain2.turretOffset = 0);
        IvyGamepads.gamepad2().dpadUp().whenBecomesTrue(() -> ShooterCalc.verticalShift += ShooterCalc.verticalShiftStep);
        IvyGamepads.gamepad2().dpadDown().whenBecomesTrue(() -> ShooterCalc.verticalShift -= ShooterCalc.verticalShiftStep);
        IvyGamepads.gamepad2().y().whenBecomesTrue(() -> DriveTrain2.toggleTurretPark());
    }

    @Override
    public void onUpdate() {

    }

    @Override
    public void onStartButtonPressed() {
    }

    public void onStop(){
        red=false;
    }
}