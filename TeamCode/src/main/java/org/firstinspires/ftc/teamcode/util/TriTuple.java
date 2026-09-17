package org.firstinspires.ftc.teamcode.util;

public class TriTuple<T1, T2, T3> extends Tuple<T1, T2> {
    public final T3 v3;

    public TriTuple(T1 val1, T2 val2, T3 val3) {
        super(val1, val2);
        this.v3 = val3;
    }
}