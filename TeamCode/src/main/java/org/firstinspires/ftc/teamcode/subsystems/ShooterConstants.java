package org.firstinspires.ftc.teamcode.subsystems;


import org.firstinspires.ftc.teamcode.ivy.*;
import org.firstinspires.ftc.teamcode.pedroPathing.*;

import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.*;
import static org.firstinspires.ftc.teamcode.ivy.HardwareCommands.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.IvyPedroCommands.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.Poses.pose;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;

@Configurable
public class ShooterConstants {
    public static Pose GOAL_POS_RED = pose(144, 144);
    public static Pose GOAL_POS_BLUE = pose(144 - GOAL_POS_RED.x(), GOAL_POS_RED.y(),
            MathFunctions.normalizeAngle(Math.PI - GOAL_POS_RED.heading()));
    public static double SCORE_HEIGHT = 32.75; //inches

    public static double SCORE_ANGLE = -17;

    public static double PASS_THROUGH_POINT_RADIUS = 0; //inches change to 4 if goon

    public static double RAISE_TIME = 0.15;

}
