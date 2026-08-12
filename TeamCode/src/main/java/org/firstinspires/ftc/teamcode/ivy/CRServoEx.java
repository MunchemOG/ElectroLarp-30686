package org.firstinspires.ftc.teamcode.ivy;

import com.qualcomm.robotcore.hardware.CRServo;

import java.util.function.Supplier;

public final class CRServoEx implements Powerable {
    private final Supplier<CRServo> supplier;

    public CRServoEx(Supplier<CRServo> supplier) { this.supplier = supplier; }
    @Override public void setPower(double power) {
        CRServo servo = supplier.get();
        if (servo == null) throw new IllegalStateException("CRServo is not initialized");
        servo.setPower(power);
    }
}
