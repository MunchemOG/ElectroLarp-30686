package org.firstinspires.ftc.teamcode.ivy;

import com.pedropathing.ivy.Command;

/** Lightweight lifecycle contract; subsystem instances are also Ivy requirements. */
public interface Subsystem {
    default void initialize() { }
    default void periodic() { }
    default void onStop() { }
    default Command getDefaultCommand() { return null; }
}
