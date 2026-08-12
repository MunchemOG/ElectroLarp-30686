package org.firstinspires.ftc.teamcode.subsystems;


import org.firstinspires.ftc.teamcode.ivy.*;
import org.firstinspires.ftc.teamcode.pedroPathing.*;

import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.*;
import static org.firstinspires.ftc.teamcode.ivy.HardwareCommands.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.IvyPedroCommands.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.Poses.pose;
import com.pedropathing.math.Pose;

public class Storage {

    public static double currentX = 120.032;

    public static double currentY = 71.163;

    public static double currentHeading = 90;

    public static Pose currentPose = pose(currentX, currentY, currentHeading);
    public static boolean setPose=false;
}