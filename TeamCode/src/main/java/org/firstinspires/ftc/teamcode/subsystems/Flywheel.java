package org.firstinspires.ftc.teamcode.subsystems;


import com.bylazar.configurables.annotations.Configurable;

import dev.nextftc.bindings.*;
import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.BasicFeedforwardParameters;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
@Configurable
public class Flywheel implements Subsystem {
    public Flywheel() {

    }

    public static final Flywheel INSTANCE = new Flywheel();
    public static double flywheelvelocity;

    public static double flywheelvelocity2;
    private ControlSystem controller1;
    private ControlSystem controller2;

    public static MotorEx flywheel = new MotorEx("launchingmotor");

    public static MotorEx flywheel2 = new MotorEx("launchingmotor2");

    public static double kS = 0.2;
    public static double kF = 0.000449;
    public static double kP = 0.009;

    public static double configvelocity = 1400; //far zone - ~1500. near zone - ~1200-1300

    public static void velocityControlWithFeedforwardExample(KineticState currentstate, float configtps) {
        ControlSystem controller1 = ControlSystem.builder()
                .velPid(new PIDCoefficients(kP, 0, 0))
                .basicFF(new BasicFeedforwardParameters(0, 0, kS))
                .build();
        controller1.setGoal(new KineticState(0.0, configtps, 0.0));
        flywheel.setPower(controller1.calculate(currentstate) + kF * configtps);
    }

    public static void velocityControlWithFeedforwardExample2(KineticState currentstate, float configtps) {
        ControlSystem controller2 = ControlSystem.builder()
                .velPid(new PIDCoefficients(kP, 0, 0))
                .basicFF(new BasicFeedforwardParameters(0, 0, kS))
                .build();
        controller2.setGoal(new KineticState(0.0, configtps, 0.0));
        flywheel2.setPower(-1 * (controller2.calculate(currentstate) + kF * configtps));
    }
    public static void shooter(float tps) {
        BindingManager.update();
        flywheelvelocity = flywheel.getVelocity();
        flywheelvelocity2 = flywheel2.getVelocity();
        KineticState currentState = new KineticState(0, flywheelvelocity, 0.0);
        KineticState currentState2 = new KineticState(0, -1*flywheelvelocity2, 0.0);
        //if(tps-(-1*flywheelvelocity)<7 && tps-(-1*flywheelvelocity)>-7){
        velocityControlWithFeedforwardExample(currentState2, tps );
        velocityControlWithFeedforwardExample2(currentState2, tps );

    }
    @Override public void initialize() {

    }

    @Override public void periodic() {

    }
}