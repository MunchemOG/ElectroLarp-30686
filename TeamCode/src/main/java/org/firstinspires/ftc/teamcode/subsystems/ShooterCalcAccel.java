package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.ShooterConstants.SCORE_ANGLE;
import static java.lang.Double.isNaN;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

@Configurable
public class ShooterCalcAccel implements Subsystem {

    //public static double farzoneangle = -0.34006585;

    //public static double farzoneheight = 10.25;


    public static double requiredRPM;

    public static double rpmoffset = 200;
    public static double requiredTPS = (28*requiredRPM)/60;
    public static double verticalShift = 0;
    public static double verticalShiftStep = 50;
    public static double sotmFactor = 1; //shld be 1.3
    public static double sotmOffset = 0;

    public static double accelScalar = 0.175; // Set to 0 to disable

    public static Double[] calculateShotVectorandUpdateHeading(double robotHeading, Vector robotToGoalVector, Vector robotVel, Vector robotAccel){
        double g = 32.174*12;
        double x = robotToGoalVector.getMagnitude()-ShooterConstants.PASS_THROUGH_POINT_RADIUS;
        //double y = -4.5745*temp*temp*temp + 25.978*temp*temp - 48.395*temp + 58.675;
        //double y = SCORE_HEIGHT;
        //double a = ShooterConstants.SCORE_ANGLE;
        double y = 0.0032*Math.pow(x,2)-0.6653*x+66.888+1;
        double a = Math.toRadians(SCORE_ANGLE);
        if(x<66.29){
            a = Math.toRadians(0.6106*x-57.478);
        } //closezone regression cuz regression is weird and cant do spline thingy
        else if(x>136){
            a = Math.toRadians(-30);
            y= 36;
        } //farzone toggle cuz i lazy


        double hoodAngle = MathFunctions.clamp(Math.atan(2 * y / x - Math.tan(a)), Math.toRadians(40),
                Math.toRadians(75));

        if(isNaN(hoodAngle)){
            hoodAngle=Math.toRadians(75);
        }
        double flywheelSpeed = Math.sqrt(g * x * x / (2 * Math.pow(Math.cos(hoodAngle), 2) * (x * Math. tan(hoodAngle) - y)));

        Vector robotVelocity = robotVel.times(1);
        //robotVelocity = new Vector(robotVel.getMagnitude(), robotVel.getTheta());
        if(robotAccel.getMagnitude() < 0) {
            robotVelocity.setMagnitude(robotVelocity.getMagnitude() + (robotAccel.getMagnitude() * accelScalar));
        }
        if (robotVelocity.getMagnitude() < 10.0 || robotAccel.getMagnitude()< -10) {
            robotVelocity = new Vector(0, 0);
        }


        double coordinateTheta = robotVelocity.getTheta() - robotToGoalVector.getTheta();

        double parallelComponent = Math.cos(coordinateTheta) * robotVelocity.getMagnitude();
        double perpendicularComponent = Math.sin(coordinateTheta) * robotVelocity.getMagnitude();

        double vz = flywheelSpeed * Math.sin(hoodAngle);
        double time = 1.175*(x / (flywheelSpeed * Math.cos(hoodAngle))); //maybe try 1.25 SIDENOTE revert back to 1.2 if accel change does NOT work, then multiply the speed itself by 1.2
        double ivr = x / time - parallelComponent;
        double nvr = Math.sqrt(ivr * ivr + perpendicularComponent * perpendicularComponent);
        double ndr = nvr * time;

        y = 0.0032*Math.pow(ndr,2)-0.6653*ndr+66.888+1;
        if(ndr<66.29){
            a = Math.toRadians(0.6106*ndr-57.478);
        } //closezone regression cuz regression is weird and cant do spline thingy
        else if(ndr>136){
            a = Math.toRadians(-30);
            y= 36;
        }

        hoodAngle = MathFunctions.clamp(Math.atan(vz / nvr), Math.toRadians(40),
                Math.toRadians(75));

        if(isNaN(hoodAngle)){
            hoodAngle=Math.toRadians(75);
        }
         //farzone toggle cuz i lazy

        //y = /*SCORE_HEIGHT*/ -4.5745*newtemp*newtemp*newtemp + 25.978*newtemp*newtemp - 48.395*newtemp + 58.675; // -0.7135x2 + 0.8315x + 33.532
        flywheelSpeed = Math.sqrt(g * ndr * ndr / (2 * Math.pow(Math.cos(hoodAngle), 2) * (ndr * Math. tan(hoodAngle) - y)));
        flywheelSpeed = flywheelSpeed/ 39.37;

        double safeIvr = Math.max(ivr, 50.0);
        double headingVelCompOffset = sotmFactor * Math.atan(perpendicularComponent / safeIvr);
        double headingAngle = Math.toDegrees(robotToGoalVector.getTheta() - robotHeading - headingVelCompOffset);

        //requiredRPM = (1.4286* Math.pow(flywheelSpeed, 3) - 39.264*Math.pow(flywheelSpeed, 2) + 863.57*flywheelSpeed-1373.9 + rpmoffset);
        //requiredTPS = (28 * requiredRPM) / 60;
        //requiredTPS = 12.79084*Math.pow(flywheelSpeed, 2)-69.40057*flywheelSpeed+849.93005;

        requiredTPS = (-16.19*Math.pow(flywheelSpeed, 2)+ 449.11*flywheelSpeed- 964.9) + verticalShift;
        double what = Math.toDegrees(hoodAngle);
        if(robotVelocity.getMagnitude()>10){
            requiredTPS=requiredTPS-sotmOffset;
        }

        //double c1 = (double) -13 /376;

        //double why = what * c1;

        //double when = (double) 475 /188;

        double hoodTime = (0.01625 * what) - 0.6;

        ActiveOpMode.telemetry().addData("distance: ", x);
        ActiveOpMode.telemetry().addData("height: ", y);
        ActiveOpMode.telemetry().addData("headingAngle", headingAngle);

        Double[] returnvalue = {requiredTPS, hoodTime, headingAngle};
        return returnvalue;
    }
}