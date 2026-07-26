package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.Flywheel.shooter;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;

@TeleOp(name = "FlywheelTuning")
@Configurable
public class FlywheelTuning extends NextFTCOpMode {
    public FlywheelTuning() {
        addComponents(
                new SubsystemComponent(Flywheel.INSTANCE/*, Intake.INSTANCE, Spindexer.INSTANCE*/),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE

        );
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