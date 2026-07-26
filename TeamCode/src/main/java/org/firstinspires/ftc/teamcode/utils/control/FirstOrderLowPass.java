package org.firstinspires.ftc.teamcode.utils.control;

public class FirstOrderLowPass {
    private Double prevVal = null;
    private Double prevDiff = null;
    private final double alpha;
    private double val;

    /**
     * Constructs a LowPassFilter with the specified alpha value.
     *
     * @param alpha The smoothing factor (0 [less than] alpha [less than] 1). A higher alpha means more smoothing.
     */
    public FirstOrderLowPass(double alpha) {
        this.alpha = alpha;
    }

    public void reset() {
        prevVal = null;
        prevDiff = null;
    }

    public double update(double newData) {
        if (prevVal == null) {
            prevVal = newData;
            return newData;
        }

        if (prevDiff == null) {
            prevDiff = newData - prevVal;
            prevVal = newData;
            return newData;
        }

        val = alpha * newData + (1 - alpha) * (prevVal + prevDiff);
        prevDiff = newData - prevVal;
        prevVal = newData;
        return getState();
    }

    public double getState() {
        return val;
    }
}
