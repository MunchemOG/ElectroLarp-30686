package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.math.Pose;

public class RobotConfig {
    public final Pose startPose;
    public final Alliance alliance;
    public final StartPosition startPosition;
    public final TeleOp teleop;
    public final RelocalizePosition relocalizePosition;
    public final Pose relocalizePose;

    public RobotConfig(StartPosition startPosition, Alliance alliance, TeleOp teleop, RelocalizePosition relocalizePosition) {
        this.startPosition - startPosition;
        this.startPose = startPosition.defaultPose;
        this.alliance = alliance;
        this.teleop = teleop;
        this.relocalizePose = relocalizePosition.defaultPose;
        this.relocalizePosition = relocalizePosition;
    }
}
