package org.firstinspires.ftc.teamcode.opModes.TeleOp;


import org.firstinspires.ftc.teamcode.ivy.*;
import org.firstinspires.ftc.teamcode.pedroPathing.*;

import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.*;
import static org.firstinspires.ftc.teamcode.ivy.HardwareCommands.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.IvyPedroCommands.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.Poses.pose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain2;
import org.firstinspires.ftc.teamcode.subsystems.ShooterCalc;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOpBlue")
public class TeleOpBlue2 extends IvyOpMode {
    public MotorEx intakeMotor;
    public MotorEx transfer;

    public TeleOpBlue2() {
        configurePedro(Constants::create, DriveTrain2.INSTANCE);
    }

    public static boolean blue;

    public static boolean isBlue(){
        return blue;
    }

    @Override
    public void onInit() {
        blue=true;
        intakeMotor = new MotorEx("intakeMotor");
        transfer = new MotorEx("transferMotor");
        IvyGamepads.gamepad1().leftTrigger().greaterThan(0.3).whenBecomesTrue(()-> intakeMotor.setPower(1))
                .whenBecomesFalse(() -> intakeMotor.setPower(0));
        IvyGamepads.gamepad1().leftTrigger().greaterThan(0.3).whenBecomesTrue(()-> transfer.setPower(1))
                .whenBecomesFalse(() -> transfer.setPower(0));

        IvyGamepads.gamepad2().leftTrigger().greaterThan(0.5).whenBecomesTrue(() -> DriveTrain2.turretOffset2 -= DriveTrain2.turretOffsetStep);
        IvyGamepads.gamepad2().rightTrigger().greaterThan(0.5).whenBecomesTrue(() -> DriveTrain2.turretOffset2 += DriveTrain2.turretOffsetStep);
        IvyGamepads.gamepad2().rightBumper().whenBecomesTrue(()->DriveTrain2.turretOffset2+= 1);
        IvyGamepads.gamepad2().leftBumper().whenBecomesTrue(()->DriveTrain2.turretOffset2-= 1);
        IvyGamepads.gamepad2().a().whenBecomesTrue(() -> DriveTrain2.turretOffset2 = 0);
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
        blue=false;
    }
}