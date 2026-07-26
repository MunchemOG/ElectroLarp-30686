package org.firstinspires.ftc.teamcode.opModes.TeleOp;

import static org.firstinspires.ftc.teamcode.subsystems.Flywheel.shooter;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;

import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

@Configurable
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Flywheel Test")
public class FlywheelTest extends NextFTCOpMode {
    public FlywheelTest() {
    }
    public static float flywheelspeed = 800;

    public static double servopos = 0;
    public static double servopos2 = 0.16;
    public MotorEx flywheel = new MotorEx("launchingmotor");
    public MotorEx flywheel2 = new MotorEx("launchingmotor2");
    public MotorEx intake = new MotorEx("intakeMotor");
    public MotorEx transfer = new MotorEx("transferMotor");
    public ServoEx hoodServo = new ServoEx("hoodServo");
    public ServoEx stopperServo = new ServoEx("stopperServo");
    @Override
    public void onInit() {







    }

    @Override
    public void onUpdate() {
        shooter(flywheelspeed);
        //flywheel.setPower(1);
        //flywheel2.setPower(-1);
        double ticksPerSecond = flywheel.getVelocity();
        double rpm = (ticksPerSecond / 28) * 60.0;
        PanelsTelemetry.INSTANCE.getTelemetry().addData("Motor RPM", rpm);
        PanelsTelemetry.INSTANCE.getTelemetry().update(telemetry);
        hoodServo.setPosition(servopos);
        stopperServo.setPosition(servopos2);
        intake.setPower(1);
        transfer.setPower(1);
    }


    @Override
    public void onStartButtonPressed() {

    }
}
