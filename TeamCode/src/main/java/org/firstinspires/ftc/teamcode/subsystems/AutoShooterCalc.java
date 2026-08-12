package org.firstinspires.ftc.teamcode.subsystems;


import org.firstinspires.ftc.teamcode.ivy.*;
import org.firstinspires.ftc.teamcode.pedroPathing.*;

import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.*;
import static org.firstinspires.ftc.teamcode.ivy.HardwareCommands.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.IvyPedroCommands.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.Poses.pose;
import static org.firstinspires.ftc.teamcode.subsystems.ShooterConstants.SCORE_HEIGHT;
import static java.lang.Double.isNaN;

import com.bylazar.configurables.annotations.Configurable;
import org.firstinspires.ftc.teamcode.pedroPathing.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.PolarVector;

@Configurable
public class AutoShooterCalc implements Subsystem {

    //public static double farzoneangle = -0.34006585;

    //public static double farzoneheight = 10.25;


    public static double requiredRPM;

    public static double requiredTPS = (28*requiredRPM)/60;
    public static double verticalShift = 0;

    //public static double accelScalar = 0.015; // Set to 0 to disable
    public static double accelScalar = 0;

    public static Double[] calculateShotVectorandUpdateHeading(double robotHeading, PolarVector robotToGoalVector, PolarVector robotVel, PolarVector robotAccel){
        double g = 32.174*12;
        double x = robotToGoalVector.getMagnitude()-ShooterConstants.PASS_THROUGH_POINT_RADIUS;
        double temp = x/39.37;
        //double y = -4.5745*temp*temp*temp + 25.978*temp*temp - 48.395*temp + 58.675;
        double y = SCORE_HEIGHT;
        double a = Math.toRadians(-23);
        if(x<50){
            a=Math.toRadians(-45);
            y=SCORE_HEIGHT + 4;
        }
        double hoodAngle = MathFunctions.clamp(Math.atan(2 * y / x - Math.tan(a)), Math.toRadians(40),
                Math.toRadians(75));

        if(isNaN(hoodAngle)){
            hoodAngle=Math.toRadians(75);
        }
        double flywheelSpeed = Math.sqrt(g * x * x / (2 * Math.pow(Math.cos(hoodAngle), 2) * (x * Math. tan(hoodAngle) - y)));

        PolarVector robotVelocity = new PolarVector(robotVel.getMagnitude(), robotVel.getTheta());

        if(robotAccel.getMagnitude() > 10) {
            robotVelocity.setMagnitude(robotVelocity.getMagnitude() + (robotAccel.getMagnitude() * accelScalar));
        }
        //robotVelocity = robotVelocity;

        double coordinateTheta = robotVelocity.getTheta() - robotToGoalVector.getTheta();

        double parallelComponent = -Math.cos(coordinateTheta) * robotVelocity.getMagnitude();
        double perpendicularComponent = Math.sin(coordinateTheta) * robotVelocity.getMagnitude();

        double vz = flywheelSpeed * Math.sin(hoodAngle);
        double time = (x / (flywheelSpeed * Math.cos(hoodAngle))); //maybe try 1.25 SIDENOTE revert back to 1.2 if accel change does NOT work, then multiply the speed itself by 1.2
        double ivr = x / time + parallelComponent;
        double nvr = Math.sqrt(ivr * ivr + perpendicularComponent * perpendicularComponent);
        double ndr = nvr * time;

        hoodAngle = MathFunctions.clamp(Math.atan(vz / nvr), Math.toRadians(40),
                Math.toRadians(75));

        if(isNaN(hoodAngle)){
            hoodAngle=Math.toRadians(75);
        }
        double newtemp = (ndr+5)/39.37;

        //y = /*SCORE_HEIGHT*/ -4.5745*newtemp*newtemp*newtemp + 25.978*newtemp*newtemp - 48.395*newtemp + 58.675; // -0.7135x2 + 0.8315x + 33.532
        flywheelSpeed = Math.sqrt(g * ndr * ndr / (2 * Math.pow(Math.cos(hoodAngle), 2) * (ndr * Math. tan(hoodAngle) - y)));
        flywheelSpeed = flywheelSpeed/ 39.37;

        double headingVelCompOffset = 1.5*Math.atan(perpendicularComponent / ivr);
        double headingAngle = Math.toDegrees(robotToGoalVector.getTheta() - robotHeading - headingVelCompOffset);

        //requiredRPM = (1.4286* Math.pow(flywheelSpeed, 3) - 39.264*Math.pow(flywheelSpeed, 2) + 863.57*flywheelSpeed-1373.9 + rpmoffset);
        //requiredTPS = (28 * requiredRPM) / 60;
        //requiredTPS = 12.79084*Math.pow(flywheelSpeed, 2)-69.40057*flywheelSpeed+849.93005;

        requiredTPS = (-16.19*Math.pow(flywheelSpeed, 2)+ 449.11*flywheelSpeed- 964.9) + verticalShift;
        if(robotVel.getMagnitude()>20){
            requiredTPS=requiredTPS+40;
        }
        double what = Math.toDegrees(hoodAngle);

        //double c1 = (double) -13 /376;

        //double why = what * c1;

        //double when = (double) 475 /188;

        double hoodTime = (0.01625 * what) - 0.6;
        RobotContext.telemetry().addData("hoodAngle", what);
        RobotContext.telemetry().addData("ballVelocity", flywheelSpeed);
        RobotContext.telemetry().addData("flywheelSpeed", requiredTPS);

        Double[] returnvalue = {requiredTPS, hoodTime, headingAngle};
        return returnvalue;
    }
}