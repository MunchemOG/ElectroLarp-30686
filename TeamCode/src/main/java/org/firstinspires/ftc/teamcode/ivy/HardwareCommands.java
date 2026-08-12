package org.firstinspires.ftc.teamcode.ivy;

import com.pedropathing.ivy.Command;

import static com.pedropathing.ivy.commands.Commands.instant;

public final class HardwareCommands {
    private HardwareCommands() { }
    public static Command power(Powerable hardware, double power) {
        return instant(() -> hardware.setPower(power));
    }
    public static Command position(Positionable hardware, double position) {
        return instant(() -> hardware.setPosition(position));
    }
}
