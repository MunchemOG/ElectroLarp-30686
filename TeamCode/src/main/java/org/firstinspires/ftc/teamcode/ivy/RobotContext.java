package org.firstinspires.ftc.teamcode.ivy;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/** Current FTC runtime context for hardware-owning subsystems and commands. */
public final class RobotContext {
    private static OpMode active;

    private RobotContext() { }

    static void attach(OpMode opMode) {
        active = opMode;
    }

    static void detach(OpMode opMode) {
        if (active == opMode) active = null;
    }

    private static OpMode requireActive() {
        if (active == null) {
            throw new IllegalStateException("Robot hardware was accessed outside an active OpMode");
        }
        return active;
    }

    public static HardwareMap hardwareMap() {
        return requireActive().hardwareMap;
    }

    public static Telemetry telemetry() {
        return requireActive().telemetry;
    }

    public static Gamepad gamepad1() {
        return requireActive().gamepad1;
    }

    public static Gamepad gamepad2() {
        return requireActive().gamepad2;
    }
}
