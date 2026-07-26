package org.firstinspires.ftc.teamcode.opModes.Auto.TRASH;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import dev.nextftc.ftc.NextFTCOpMode;


@Autonomous
@Configurable
@Disabled
public class  WRpath extends NextFTCOpMode {

    public class Paths {
        public PathChain launch1SpikeRow1;
        public PathChain launchRow1;
        public PathChain collectRow2;
        public PathChain launchRow2;
        public PathChain openGate1;
        public PathChain gateIntake1;
        public PathChain launchGate1;
        public PathChain gateIntake2;
        public PathChain launchGate2;
        public PathChain gateIntake3;
        public PathChain launchGate3;
        public PathChain spike3;
        public PathChain farLaunch;
        public PathChain farIntake1;
        public PathChain farLaunch1;
        public PathChain farIntake2;
        public PathChain farLaunch2;
        public PathChain farIntake3;
        public PathChain farLaunch3;
        public PathChain Park;

        public Paths(Follower follower) {
            launch1SpikeRow1 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(18.272, 113.712),
                                    new Pose(28.533, 100.685),
                                    new Pose(21.634, 79.704)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(270))
                    .build();

            launchRow1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(21.634, 79.704),
                                    new Pose(44.576, 85.210)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(270))
                    .build();

            collectRow2 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(44.576, 85.210),
                                    new Pose(58.251, 67.241),
                                    new Pose(23.101, 59.568)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))
                    .build();

            launchRow2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(23.101, 59.568),
                                    new Pose(44.451, 85.198)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            openGate1 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(44.451, 85.198),
                                    new Pose(38.000, 67.000),
                                    new Pose(11.500, 66.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(169))
                    .build();

            gateIntake1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(11.500, 66.000),
                                    new Pose(10.000, 58.750)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(170), Math.toRadians(143))
                    .build();

            launchGate1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(10.000, 58.750),
                                    new Pose(44.451, 85.198)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(175))
                    .setReversed()
                    .build();

            gateIntake2 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(44.451, 85.198),
                                    new Pose(38.000, 67.000),
                                    new Pose(10.000, 62.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(175), Math.toRadians(147))
                    .build();

            launchGate2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(10.000, 62.000),
                                    new Pose(43.704, 85.946)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(147), Math.toRadians(175))
                    .build();

            gateIntake3 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(43.704, 85.946),
                                    new Pose(10.000, 62.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(175), Math.toRadians(147))
                    .build();

            launchGate3 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(10.000, 62.000),
                                    new Pose(44.043, 85.800)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(147), Math.toRadians(270))
                    .build();

            spike3 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(44.043, 85.800),
                                    new Pose(20.086, 79.535),
                                    new Pose(23.728, 58.177),
                                    new Pose(23.549, 38.253)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(270))
                    .build();

            farLaunch = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(23.549, 38.253),
                                    new Pose(40.204, 8.605),
                                    new Pose(59.661, 9.136)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))
                    .setReversed()
                    .build();

            farIntake1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(59.661, 9.136),
                                    new Pose(12.747, 8.973)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            farLaunch1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(12.747, 8.973),
                                    new Pose(59.572, 9.136)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            farIntake2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(59.572, 9.136),
                                    new Pose(12.747, 8.973)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            farLaunch2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(12.747, 8.973),
                                    new Pose(59.572, 9.136)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            farIntake3 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(59.572, 9.136),
                                    new Pose(9.198, 8.973)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            farLaunch3 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(9.198, 8.973),
                                    new Pose(59.572, 9.136)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            Park = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(59.572, 9.136),
                                    new Pose(43.949, 14.385)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                    .build();
        }
    }



}