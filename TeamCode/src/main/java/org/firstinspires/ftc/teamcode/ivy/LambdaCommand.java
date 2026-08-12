package org.firstinspires.ftc.teamcode.ivy;

import com.pedropathing.ivy.CommandBuilder;

/**
 * Source-compatible named command builder for migrated OpModes.
 * Ivy commands do not carry display names, so the name is retained only for debugging.
 */
public final class LambdaCommand extends CommandBuilder {
    private final String name;

    public LambdaCommand(String name) {
        this.name = name;
    }

    @Override
    public String toString() {
        return name;
    }
}
