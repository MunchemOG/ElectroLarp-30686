package org.firstinspires.ftc.teamcode.util;


import static com.pedropathing.utils.Utils.clamp;

public class ScalingLowPassFilter extends Filter {
    public double coeff;
    public double factor;
    public double scaleFactor;

    private Double lastVal;
    private Double rawVal;

    public ScalingLowPassFilter(double coeff, double factor, double scaleFactor) {
        this.coeff = coeff;
        this.factor = factor;
        this.scaleFactor = scaleFactor;
    }

    public ScalingLowPassFilter(TriTuple<Double, Double, Double> values) {
        this(values.v1, values.v2, values.v3);
    }

    public ScalingLowPassFilter(double factor, double scaleFactor) {
        this(0.5, factor, scaleFactor);
    }

    public ScalingLowPassFilter(double scaleFactor) {
        this(0.5, 1, scaleFactor);
    }

    public void setValues(double coeff, double factor, double scaleFactor) {
        setCoefficient(coeff);
        setFactor(factor);
        setScaleFactor(scaleFactor);
    }

    public void setValues(TriTuple<Double, Double, Double> values) {
        setValues(values.v1, values.v2, values.v3);
    }

    public void setCoefficient(double coeff) {
        this.coeff = clamp(coeff, 0, 1);
    }

    public void setFactor(double factor) {
        this.factor = factor;
    }

    public void setScaleFactor(double scaleFactor) {
        this.scaleFactor = scaleFactor;
    }

    public double update(double value) {
        rawVal = value;

        if (lastVal == null || Double.isNaN(lastVal)) {
            lastVal = value;
            return value;
        }

        double origLastVal = lastVal;

        double newVal = lastVal + (Math.pow(Math.abs(value - lastVal), factor + Math.pow(1 + Math.abs(scaleFactor), Math.max(value, lastVal)) * Math.signum(scaleFactor)) * Math.signum(value - lastVal) * coeff);
        if (!Double.isNaN(newVal)) {
            lastVal = newVal;

            if (value < origLastVal && lastVal < value) lastVal = value;
            if (value > origLastVal && lastVal > value) lastVal = value;
        }

        return lastVal;
    }

    public double getValue() {
        return lastVal;
    }

    public double getRawValue() {
        return rawVal;
    }

    @Override
    public void forceSet(double value) {
        rawVal = value;
        lastVal = value;
    }
}