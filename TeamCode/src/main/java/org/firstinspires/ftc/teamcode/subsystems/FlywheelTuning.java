package org.firstinspires.ftc.teamcode.subsystems;


import org.firstinspires.ftc.teamcode.ivy.*;
import org.firstinspires.ftc.teamcode.pedroPathing.*;

import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.*;
import static org.firstinspires.ftc.teamcode.ivy.HardwareCommands.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.IvyPedroCommands.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.Poses.pose;
import static org.firstinspires.ftc.teamcode.subsystems.Flywheel.shooter;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "FlywheelTuning")
@Configurable
public class FlywheelTuning extends IvyOpMode {
    public FlywheelTuning() {
        configurePedro(null, Flywheel.INSTANCE/*, Intake.INSTANCE, Spindexer.INSTANCE*/);
    }

    public static MotorEx flywheel = new MotorEx("launchingmotor2");

    public static float flywheelRPM = 800;


    @Override
    public void onInit() {

    }

    @Override
    public void onUpdate() {
        shooter(flywheelRPM);
        double ticksPerSecond = flywheel.getVelocity();
        double rpm = (ticksPerSecond / 28) * 60.0;
        PanelsTelemetry.INSTANCE.getTelemetry().addData("Motor RPM", rpm);
        PanelsTelemetry.INSTANCE.getTelemetry().update(telemetry);

    }


    @Override
    public void onStartButtonPressed() {
    }
}