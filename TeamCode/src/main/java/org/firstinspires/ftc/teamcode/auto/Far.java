package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.CommandBuilder;


import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.auto.paths.FarPaths;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.CommandOpMode;
import org.psilynx.psikit.core.Logger;

/** Autonomous entry point for the close-side starting position. */
@Configurable
public class Far extends CommandOpMode {
    Alliance a;
    Robot robot;
    Follower follower;
    public Far(Alliance a) {this.a = a;}
    public void init() {
        Logger.recordMetadata("Alliance", a.name());
        Logger.recordMetadata("OpMode", "Close");
        telemetry.addData("Pose", robot.follower.pose());
        telemetry.update();
        schedule();
    }

    @Override
    public void start() {

    }
    private CommandBuilder goon() {
        return null;
    }
    public void stop(){

    }
}
