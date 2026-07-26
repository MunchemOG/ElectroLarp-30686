package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public class ShooterConstants {
    public static Pose GOAL_POS_RED = new Pose(144, 144);
    public static Pose GOAL_POS_BLUE = GOAL_POS_RED.mirror();
    public static double SCORE_HEIGHT = 32.75; //inches

    public static double SCORE_ANGLE = -17;

    public static double PASS_THROUGH_POINT_RADIUS = 0; //inches change to 4 if goon

    public static double RAISE_TIME = 0.15;

}
