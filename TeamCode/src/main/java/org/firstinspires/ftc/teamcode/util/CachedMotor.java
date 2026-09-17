package org.firstinspires.ftc.teamcode.util;

import static com.pedropathing.utils.Utils.clamp;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import java.util.HashMap;

@Configurable
public class CachedMotor implements DcMotorEx {
    public static double filterCoefficient = 1; // 0.00000002
    public static double filterFactor = 4;
    public static double filterScaleFactor = -0.000005;

    private static final HashMap<String, DcMotorEx> created = new HashMap<>();

    public final DcMotorEx motor;

    public boolean onlyQuickPower = false;

    private double targetVelocity;
    private int targetPosition;
    private double power;

    private RunMode mode;
    private ZeroPowerBehavior zeroPowerBehavior;
    private PIDFCoefficients velocityPIDFCoefficients;
    private PIDFCoefficients positionPIDFCoefficients;
    private int targetPositionTolerance;
    private Direction direction;

    private int currentCacheTimeMS = 200;
    private double lastCurrentMA = 0;
    private long lastCurrentTimestamp = 0;

    private final ScalingLowPassFilter currentFilter = new ScalingLowPassFilter(filterCoefficient, filterFactor, filterScaleFactor);

    public CachedMotor(DcMotorEx motor) {
        this.motor = motor;

        _init();
    }


    public CachedMotor(HardwareMap hardwareMap, String name) {
//        if (CachedMotor.created.get(name) != null) motor = CachedMotor.created.get(name);
//        else {
        this.motor = hardwareMap.get(DcMotorEx.class, name);

        CachedMotor.created.put(name, motor);
//        }

        _init();
    }

    private void _init() {
        this.targetVelocity = 0;
        this.targetPosition = motor.getTargetPosition();
        this.power = motor.getPower();

        this.mode = motor.getMode();
        this.zeroPowerBehavior = motor.getZeroPowerBehavior();
        this.velocityPIDFCoefficients = motor.getPIDFCoefficients(RunMode.RUN_USING_ENCODER);
        this.positionPIDFCoefficients = motor.getPIDFCoefficients(RunMode.RUN_TO_POSITION);
        this.targetPositionTolerance = motor.getTargetPositionTolerance();
        this.direction = motor.getDirection();
    }

    public void setOnlyQuickPower(boolean onlyQuickPower) {
        this.onlyQuickPower = onlyQuickPower;
    }

    @Override
    public void setMotorEnable() {
        motor.setMotorEnable();
    }

    @Override
    public void setMotorDisable() {
        motor.setMotorDisable();
    }

    @Override
    public boolean isMotorEnabled() {
        return motor.isMotorEnabled();
    }

    @Override
    public void setVelocity(double angularRate) {
        if (this.targetVelocity == angularRate) return;
        this.targetVelocity = angularRate;
        motor.setVelocity(angularRate);
    }

    @Override
    @Deprecated
    public void setVelocity(double angularRate, AngleUnit unit) {
        motor.setVelocity(angularRate, unit);
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    @Override
    public double getVelocity() {
        return motor.getVelocity();
    }

    @Override
    @Deprecated
    public double getVelocity(AngleUnit unit) {
        return motor.getVelocity(unit);
    }

    @Override
    @Deprecated
    public void setPIDCoefficients(RunMode mode, PIDCoefficients pidCoefficients) {
        PIDFCoefficients coeff = getPIDFCoefficients(mode);
        this.setPIDFCoefficients(mode, new PIDFCoefficients(pidCoefficients.p, pidCoefficients.i, pidCoefficients.d, coeff.f));
    }

    @Override
    public void setPIDFCoefficients(RunMode mode, PIDFCoefficients pidfCoefficients) throws UnsupportedOperationException {
        switch (mode) {
            case RUN_TO_POSITION:
                this.positionPIDFCoefficients = pidfCoefficients;
                break;
            case RUN_USING_ENCODER:
                this.velocityPIDFCoefficients = pidfCoefficients;
                break;
        }

        motor.setPIDFCoefficients(mode, pidfCoefficients);
    }

    public void setVelocityPIDFCoefficients(PIDFCoefficients pidfCoefficients) {
        this.setPIDFCoefficients(RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }

    @Override
    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
        this.setPIDFCoefficients(RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, i, d, f));
    }

    @Override
    @Deprecated
    public void setPositionPIDFCoefficients(double p) {
        PIDFCoefficients coeff = getPIDFCoefficients(RunMode.RUN_TO_POSITION);
        this.setPIDFCoefficients(RunMode.RUN_TO_POSITION, new PIDFCoefficients(p, coeff.i, coeff.d, coeff.f));
    }

    public void setPositionPIDFCoefficients(PIDFCoefficients pidfCoefficients) {
        this.setPIDFCoefficients(RunMode.RUN_TO_POSITION, pidfCoefficients);
    }

    public void setPositionPIDFCoefficients(double p, double i, double d, double f) {
        this.setPIDFCoefficients(RunMode.RUN_TO_POSITION, new PIDFCoefficients(p, i, d, f));
    }

    @Override
    @Deprecated
    public PIDCoefficients getPIDCoefficients(RunMode mode) {
        PIDFCoefficients coeff = this.getPIDFCoefficients(mode);
        return new PIDCoefficients(coeff.p, coeff.i, coeff.d);
    }

    @Override
    public PIDFCoefficients getPIDFCoefficients(RunMode mode) {
        switch (mode) {
            case RUN_TO_POSITION:
                return positionPIDFCoefficients;
            case RUN_USING_ENCODER:
                return velocityPIDFCoefficients;

            default:
                return null;
        }
    }

    public PIDFCoefficients getVelocityPIDFCoefficients() {
        return velocityPIDFCoefficients;
    }

    public PIDFCoefficients getPositionPIDFCoefficients() {
        return positionPIDFCoefficients;
    }

    @Override
    public void setTargetPositionTolerance(int tolerance) {
        this.targetPositionTolerance = tolerance;
        motor.setTargetPositionTolerance(tolerance);
    }

    @Override
    public int getTargetPositionTolerance() {
        return targetPositionTolerance;
    }

    public void setCurrentCacheTime(int currentCacheTimeMS) {
        this.currentCacheTimeMS = currentCacheTimeMS;
    }

    public void setCurrentFilter(double coeff, double factor, double scaleFactor) {
        currentFilter.setValues(coeff, factor, scaleFactor);
    }

    public void setCurrentFilter(TriTuple<Double, Double, Double> values) {
        currentFilter.setValues(values);
    }

    @Override
    public double getCurrent(CurrentUnit unit) {
        double valMA;

        if (lastCurrentTimestamp + currentCacheTimeMS > System.currentTimeMillis()) {
            valMA = lastCurrentMA;
        } else {
            valMA = motor.getCurrent(CurrentUnit.MILLIAMPS);
            lastCurrentMA = valMA;
            lastCurrentTimestamp = System.currentTimeMillis();
        }

        currentFilter.setValues(filterCoefficient, filterFactor, filterScaleFactor);
        double val = currentFilter.update(valMA);

        switch (unit) {
            case AMPS:
                return val / 1000;

            case MILLIAMPS:
                return val ;

            default:
                return 0;
        }
    }

    public double getRawCurrent(CurrentUnit unit) {
        double valMA;

        if (lastCurrentTimestamp + currentCacheTimeMS > System.currentTimeMillis()) {
            valMA = lastCurrentMA;
        } else {
            valMA = motor.getCurrent(CurrentUnit.MILLIAMPS);
            lastCurrentMA = valMA;
            lastCurrentTimestamp = System.currentTimeMillis();
        }

        currentFilter.setValues(filterCoefficient, filterFactor, filterScaleFactor);
        currentFilter.update(valMA);

        switch (unit) {
            case AMPS:
                return valMA * 1000;

            case MILLIAMPS:
                return valMA;

            default:
                return 0;
        }
    }

    @Override
    public double getCurrentAlert(CurrentUnit unit) {
        return motor.getCurrentAlert(unit);
    }

    @Override
    public void setCurrentAlert(double current, CurrentUnit unit) {
        motor.setCurrentAlert(current, unit);
    }

    @Override
    public boolean isOverCurrent() {
        return motor.isOverCurrent();
    }

    @Override
    public MotorConfigurationType getMotorType() {
        return motor.getMotorType();
    }

    @Override
    public void setMotorType(MotorConfigurationType motorType) {
        motor.setMotorType(motorType);
    }

    @Override
    public DcMotorController getController() {
        return motor.getController();
    }

    @Override
    public int getPortNumber() {
        return motor.getPortNumber();
    }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        setZeroPowerBehavior(zeroPowerBehavior, false);
    }

    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior, boolean force) {
        if (!force && this.zeroPowerBehavior == zeroPowerBehavior) return;
        this.zeroPowerBehavior = zeroPowerBehavior;
        motor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return zeroPowerBehavior;
    }

    @Override
    @Deprecated
    public void setPowerFloat() {
        motor.setPowerFloat();
    }

    @Override
    public boolean getPowerFloat() {
        return motor.getPowerFloat();
    }

    @Override
    public void setTargetPosition(int position) {
        if (targetPosition == position) return;
        this.targetPosition = position;
        motor.setTargetPosition(position);
    }

    @Override
    public int getTargetPosition() {
        return targetPosition;
    }

    @Override
    public boolean isBusy() {
        return motor.isBusy();
    }

    @Override
    public int getCurrentPosition() {
        return motor.getCurrentPosition();
    }

    @Override
    public void setMode(RunMode mode) {
        this.mode = mode;
        motor.setMode(mode);
    }

    @Override
    public RunMode getMode() {
        return mode;
    }

    public void resetEncoder() {
//        try {
//            new LynxResetMotorEncoderCommand(motor., motor.getPortNumber()).send();
//        } catch (InterruptedException | LynxNackException e) {
//        }
        motor.setMode(RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(mode);
    }

    @Override
    public void setDirection(Direction direction) {
        this.direction = direction;
        motor.setDirection(direction);
    }

    public void setInverted(boolean inverted) {
        setDirection(inverted ? Direction.REVERSE : Direction.FORWARD);
    }

    @Override
    public Direction getDirection() {
        return direction;
    }

    public boolean getInverted() {
        return getDirection() == Direction.REVERSE;
    }

    @Override
    public void setPower(double power) {
        power = clamp(power, -1, 1);
        if (Math.abs(this.power - power) < 0.01) return;
        this.power = power;
        motor.setPower(power);
    }

    @Override
    public double getPower() {
        return power;
    }

    @Override
    public Manufacturer getManufacturer() {
        return motor.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return motor.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return motor.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return motor.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        motor.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        motor.close();
    }
}