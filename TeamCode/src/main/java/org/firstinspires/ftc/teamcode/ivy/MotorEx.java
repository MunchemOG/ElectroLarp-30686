package org.firstinspires.ftc.teamcode.ivy;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/** Lazy SDK motor wrapper; construction is safe during OpMode discovery. */
public final class MotorEx implements Powerable {
    private final String name;
    private boolean reversed;
    private boolean brake;
    private DcMotorEx motor;

    public MotorEx(String name) { this.name = name; }

    public MotorEx reversed() {
        reversed = !reversed;
        if (motor != null) applyConfiguration();
        return this;
    }

    public MotorEx brakeMode() {
        brake = true;
        if (motor != null) applyConfiguration();
        return this;
    }

    private DcMotorEx motor() {
        if (motor == null) {
            motor = RobotContext.hardwareMap().get(DcMotorEx.class, name);
            applyConfiguration();
        }
        return motor;
    }

    private void applyConfiguration() {
        motor.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(brake ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override public void setPower(double power) { motor().setPower(power); }
    public double getPower() { return motor().getPower(); }
    public double getVelocity() { return motor().getVelocity(); }
    public int getCurrentPosition() { return motor().getCurrentPosition(); }
    public DcMotorEx sdkMotor() { return motor(); }
}
