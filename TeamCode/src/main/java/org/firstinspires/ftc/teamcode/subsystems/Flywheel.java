package org.firstinspires.ftc.teamcode.subsystems;



import org.firstinspires.ftc.teamcode.ivy.*;
import org.firstinspires.ftc.teamcode.pedroPathing.*;

import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.*;
import static org.firstinspires.ftc.teamcode.ivy.HardwareCommands.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.IvyPedroCommands.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.Poses.pose;
import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class Flywheel implements Subsystem {
    public Flywheel() {

    }

    public static final Flywheel INSTANCE = new Flywheel();
    public static double flywheelvelocity;

    public static double flywheelvelocity2;
    public static MotorEx flywheel = new MotorEx("launchingmotor");

    public static MotorEx flywheel2 = new MotorEx("launchingmotor2");

    public static double kS = 0.2;
    public static double kF = 0.000449;
    public static double kP = 0.009;

    public static double configvelocity = 1400; //far zone - ~1500. near zone - ~1200-1300

    private static double controlPower(double measuredTps, double targetTps) {
        if (Math.abs(targetTps) < 1e-6) return 0;
        double power = Math.copySign(kS, targetTps) + kF * targetTps
                + kP * (targetTps - measuredTps);
        return Math.max(-1, Math.min(1, power));
    }

    public static void velocityControlWithFeedforwardExample(double measuredTps, float targetTps) {
        flywheel.setPower(controlPower(measuredTps, targetTps));
    }

    public static void velocityControlWithFeedforwardExample2(double measuredTps, float targetTps) {
        flywheel2.setPower(-controlPower(measuredTps, targetTps));
    }
    public static void shooter(float tps) {
        flywheelvelocity = flywheel.getVelocity();
        flywheelvelocity2 = flywheel2.getVelocity();
        velocityControlWithFeedforwardExample(flywheelvelocity, tps);
        velocityControlWithFeedforwardExample2(-flywheelvelocity2, tps);

    }
    @Override public void initialize() {

    }

    @Override public void periodic() {

    }
}
