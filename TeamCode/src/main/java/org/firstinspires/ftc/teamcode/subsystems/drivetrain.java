package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class drivetrain {

    final String frHW = "";
    final String flHW = "";
    final String brHW = "";
    final String blHW = "";

    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;

    drivetrain(HardwareMap hw) {
        frontLeft = hw.get(DcMotorEx.class, flHW);
        frontRight = hw.get(DcMotorEx.class, frHW);
        backLeft = hw.get(DcMotorEx.class, blHW);
        backRight = hw.get(DcMotorEx.class, brHW);
    }

    public void drive(double forward, double strafe, double spinRate, double heading) {
        double magnitude = Math.min(1.0, Math.hypot(forward, strafe));
        double direction = Math.atan2(strafe, forward);
        double forwardCartesian = magnitude * Math.cos(direction);
        double strafeCartesian = magnitude * Math.sin(direction);

        double cosine = Math.cos(heading), sine = Math.sin(heading);
        double forwardVelocity = forwardCartesian * cosine + strafeCartesian * sine;
        double strafeVelocity = -forwardCartesian * sine + strafeCartesian * cosine;

        double fl = forwardVelocity - strafeVelocity - spinRate;
        double fr = forwardVelocity + strafeVelocity + spinRate;
        double bl = forwardVelocity + strafeVelocity - spinRate;
        double br = forwardVelocity - strafeVelocity + spinRate;

        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

}
