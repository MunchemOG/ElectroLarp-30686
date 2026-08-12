package org.firstinspires.ftc.teamcode.ivy;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/** Small edge-trigger binding helper for Ivy OpModes. */
public final class IvyGamepads {
    private static final List<BooleanBinding> bindings = new ArrayList<>();

    private IvyGamepads() { }

    public static Pad gamepad1() { return new Pad(RobotContext::gamepad1); }
    public static Pad gamepad2() { return new Pad(RobotContext::gamepad2); }

    static void update() {
        for (BooleanBinding binding : new ArrayList<>(bindings)) binding.update();
    }

    static void reset() {
        bindings.clear();
    }

    public static final class Pad {
        private final java.util.function.Supplier<Gamepad> gamepad;

        private Pad(java.util.function.Supplier<Gamepad> gamepad) { this.gamepad = gamepad; }

        public DoubleBinding leftTrigger() { return new DoubleBinding(() -> gamepad.get().left_trigger); }
        public DoubleBinding rightTrigger() { return new DoubleBinding(() -> gamepad.get().right_trigger); }
        public DoubleBinding leftStickX() { return new DoubleBinding(() -> gamepad.get().left_stick_x); }
        public DoubleBinding leftStickY() { return new DoubleBinding(() -> gamepad.get().left_stick_y); }
        public DoubleBinding rightStickX() { return new DoubleBinding(() -> gamepad.get().right_stick_x); }
        public BooleanBinding a() { return register(() -> gamepad.get().a); }
        public BooleanBinding b() { return register(() -> gamepad.get().b); }
        public BooleanBinding x() { return register(() -> gamepad.get().x); }
        public BooleanBinding y() { return register(() -> gamepad.get().y); }
        public BooleanBinding leftBumper() { return register(() -> gamepad.get().left_bumper); }
        public BooleanBinding rightBumper() { return register(() -> gamepad.get().right_bumper); }
        public BooleanBinding dpadUp() { return register(() -> gamepad.get().dpad_up); }
        public BooleanBinding dpadDown() { return register(() -> gamepad.get().dpad_down); }

        private static BooleanBinding register(BooleanSupplier supplier) {
            BooleanBinding binding = new BooleanBinding(supplier);
            bindings.add(binding);
            return binding;
        }
    }

    public static final class DoubleBinding implements DoubleSupplier {
        private final DoubleSupplier supplier;

        private DoubleBinding(DoubleSupplier supplier) { this.supplier = supplier; }

        public BooleanBinding greaterThan(double threshold) {
            BooleanBinding binding = new BooleanBinding(() -> getAsDouble() > threshold);
            bindings.add(binding);
            return binding;
        }

        @Override public double getAsDouble() { return supplier.getAsDouble(); }
    }

    public static final class BooleanBinding {
        private final BooleanSupplier supplier;
        private boolean initialized;
        private boolean previous;
        private Runnable onTrue = () -> { };
        private Runnable onFalse = () -> { };

        private BooleanBinding(BooleanSupplier supplier) { this.supplier = supplier; }

        public BooleanBinding whenBecomesTrue(Runnable action) { onTrue = action; return this; }
        public BooleanBinding whenBecomesTrue(Command command) { return whenBecomesTrue(command::schedule); }
        public BooleanBinding whenBecomesFalse(Runnable action) { onFalse = action; return this; }
        public BooleanBinding whenBecomesFalse(Command command) { return whenBecomesFalse(command::schedule); }

        private void update() {
            boolean current = supplier.getAsBoolean();
            if (!initialized) {
                initialized = true;
                previous = current;
                return;
            }
            if (current && !previous) onTrue.run();
            if (!current && previous) onFalse.run();
            previous = current;
        }
    }
}
