package org.firstinspires.ftc.teamcode.opModes.TeleOp;

import static org.firstinspires.ftc.teamcode.subsystems.DriveTrain2.intakeMotor;
import static org.firstinspires.ftc.teamcode.subsystems.DriveTrain2.transfer;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain2;
import org.firstinspires.ftc.teamcode.subsystems.ShooterCalc;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOpRedCRI")
public class TeleOpRed2 extends NextFTCOpMode {
    //public MotorEx intakeMotor;
    //public MotorEx transfer;

    public TeleOpRed2() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(DriveTrain2.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
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
        Gamepads.gamepad1().leftTrigger().greaterThan(0.3)
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
        Gamepads.gamepad2().leftTrigger().greaterThan(0.5).whenBecomesTrue(() -> DriveTrain2.turretOffset -= DriveTrain2.turretOffsetStep);
        Gamepads.gamepad2().rightTrigger().greaterThan(0.5).whenBecomesTrue(() -> DriveTrain2.turretOffset += DriveTrain2.turretOffsetStep);
        Gamepads.gamepad2().rightBumper().whenBecomesTrue(()->DriveTrain2.turretOffset+= 1);
        Gamepads.gamepad2().leftBumper().whenBecomesTrue(()->DriveTrain2.turretOffset-= 1);
        Gamepads.gamepad2().a().whenBecomesTrue(() -> DriveTrain2.turretOffset = 0);
        Gamepads.gamepad2().dpadUp().whenBecomesTrue(() -> ShooterCalc.verticalShift += ShooterCalc.verticalShiftStep);
        Gamepads.gamepad2().dpadDown().whenBecomesTrue(() -> ShooterCalc.verticalShift -= ShooterCalc.verticalShiftStep);
        Gamepads.gamepad2().y().whenBecomesTrue(() -> DriveTrain2.toggleTurretPark());
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