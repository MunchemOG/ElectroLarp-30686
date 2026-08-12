package org.firstinspires.ftc.teamcode.ivy;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.PedroRuntime;
import org.firstinspires.ftc.teamcode.pedroPathing.TeamFollower;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Function;

/** FTC iterative OpMode lifecycle wired to Ivy's static Scheduler. */
public abstract class IvyOpMode extends OpMode {
    private Function<HardwareMap, TeamFollower> followerFactory;
    private final List<Subsystem> subsystems = new ArrayList<>();
    private final List<Command> defaultCommands = new ArrayList<>();

    protected final void configurePedro(Function<HardwareMap, TeamFollower> factory,
                                        Subsystem... configuredSubsystems) {
        followerFactory = factory;
        subsystems.clear();
        subsystems.addAll(Arrays.asList(configuredSubsystems));
    }

    @Override
    public final void init() {
        RobotContext.attach(this);
        IvyGamepads.reset();
        Scheduler.reset();
        if (followerFactory != null) PedroRuntime.attach(followerFactory.apply(hardwareMap));
        for (Subsystem subsystem : subsystems) {
            subsystem.initialize();
            Command defaultCommand = subsystem.getDefaultCommand();
            if (defaultCommand != null) defaultCommands.add(defaultCommand);
        }
        onInit();
    }

    @Override
    public final void init_loop() {
        IvyGamepads.update();
        Scheduler.execute();
        onInitLoop();
    }

    @Override
    public final void start() {
        for (Command command : defaultCommands) Scheduler.schedule(command);
        onStartButtonPressed();
    }

    @Override
    public final void loop() {
        IvyGamepads.update();
        for (Subsystem subsystem : subsystems) subsystem.periodic();
        onUpdate();
        Scheduler.execute();
    }

    @Override
    public final void stop() {
        try {
            onStop();
            for (Subsystem subsystem : subsystems) subsystem.onStop();
        } finally {
            Scheduler.reset();
            PedroRuntime.detach();
            IvyGamepads.reset();
            RobotContext.detach(this);
            defaultCommands.clear();
        }
    }

    public void onInit() { }
    public void onInitLoop() { }
    public void onStartButtonPressed() { }
    public void onUpdate() { }
    public void onStop() { }
}
