package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public abstract class CommandOpMode extends OpMode {
    public void reset() {
        Scheduler.reset();
    }

    public void schedule(Command... commands) {
        Scheduler.schedule(commands);
    }

    @Override
    public void init() {}

    @Override
    public void loop() {
        Scheduler.execute();
    }

    public void stop() {
        reset();
    }
}
