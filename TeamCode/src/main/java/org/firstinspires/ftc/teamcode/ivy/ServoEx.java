package org.firstinspires.ftc.teamcode.ivy;

import com.qualcomm.robotcore.hardware.Servo;

import java.util.function.Supplier;

/** Lazy SDK positional-servo wrapper. */
public final class ServoEx implements Positionable {
    private final String name;
    private final Supplier<Servo> supplier;
    private Servo servo;

    public ServoEx(String name) { this.name = name; this.supplier = null; }
    public ServoEx(Supplier<Servo> supplier) { this.name = null; this.supplier = supplier; }

    private Servo servo() {
        if (servo == null) {
            servo = supplier != null ? supplier.get() : RobotContext.hardwareMap().get(Servo.class, name);
            if (servo == null) throw new IllegalStateException("Servo is not initialized: " + name);
        }
        return servo;
    }

    @Override public void setPosition(double position) { servo().setPosition(position); }
    public double getPosition() { return servo().getPosition(); }
}
