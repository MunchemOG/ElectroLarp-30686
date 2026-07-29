package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.opModes.TeleOp.TeleOpBlue2.isBlue;
import static org.firstinspires.ftc.teamcode.opModes.TeleOp.TeleOpRed2.isRed;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
import static org.firstinspires.ftc.teamcode.subsystems.Flywheel.shooter;
import static org.firstinspires.ftc.teamcode.subsystems.LaunchDetector.isOverlappingLaunchZone;
import static org.firstinspires.ftc.teamcode.subsystems.ShooterCalcAccelClaude.calculateShotVectorandUpdateHeading;
import static org.firstinspires.ftc.teamcode.subsystems.ShooterCalcAccelClaude.requiredTPS;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.io.FileWriter;
import java.util.List;
import java.util.function.Supplier;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.core.units.Angle;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.hardware.driving.FieldCentric;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;


@Configurable
public class DriveTrain2 implements Subsystem {

    public static final DriveTrain2 INSTANCE = new DriveTrain2();

    public DriveTrain2() {
    }


    public static MotorEx fL, fR, bL, bR;

    private IMUEx imu;
    public static double tolarance = 10;
    public double currentMotorSpeed = 0;

    public boolean firsttime = true;

    public static double servoOffset = 0.0025;
    public static double wrapServoOffset = 0.0;
    public static double turretServoThreshold = 0.0012;

    // ---- Heading lock (hold gamepad1 LEFT BUMPER) ---------------------------
    // Ported from Meta Infinity's RobotTeleop. While held, the right stick is
    // ignored and the drivetrain rotates itself to a fixed field heading.
    // The turret keeps tracking the goal regardless, since its angle is
    // recomputed from the live robot heading every loop.
    //
    // Useful side effect for this robot: locking the chassis heading also
    // shrinks the range of turret angles ever demanded, which is the cheapest
    // way to stop provoking wraps in the first place.

    /** Master enable. */
    public static boolean headingLockEnabled = true;

    /** Target field heading per alliance, degrees. THESE ARE PLACEHOLDERS -
     *  they are Meta Infinity's values and describe THEIR field positions.
     *  Set them to whatever you actually want to snap to. */
    public static double headingLockRedDeg = 36;
    public static double headingLockBlueDeg = 144.0;

    /** Output clamp - caps how hard the lock can spin the robot. */
    public static double headingLockMaxPower = 0.5;

    /** Inside this error, command zero, so it stops hunting on the target. */
    public static double headingLockDeadbandDeg = 2.0;

    /** Proportional gain, power per degree of error. */
    public static double headingKp = 0.001;

    /** Static feedforward: a constant push in the error direction, to break
     *  stiction that the P term alone cannot overcome close to the target. */
    public static double headingKs = 0.1;

    /** Flip to -1.0 if the robot rotates AWAY from the target instead of
     *  toward it. Their rotation sign convention may not match this drive
     *  mapping, and this is the one-value fix. TEST ON BLOCKS FIRST. */
    public static double headingLockSign = 1.0;

    /** True while the lock is driving rotation. Telemetry. */
    public static boolean headingLockActive = false;

    /** Bumper state, maintained by the binding registered in periodic(). */
    private boolean headingLockHeld = false;

    public static double yawFeedforwardGain = 0.115;

    //TURN OFF IN MATCHES
    public static boolean logAccel = false;

    /** Command-rate instrumentation ONLY. This build does not apply the
     *  command-rate feedforward to the turret; cmdRate is measured and logged
     *  so the lag question can be answered from data before any change. */
    public static double cmdRateFilterAlpha = 0.3;
    public static double cmdRateSpikeLimit = 400.0;

    public static double maxSlewNormalDegPerSec = 300;

    public static double maxSlewWrapDegPerSec = 300;

    public static boolean turretParked = false;

    public static double turretParkSignal = 0.5;
    public static void toggleTurretPark() {
        turretParked = !turretParked;
    }

    private double slewedTurretAngle = Double.NaN;
    private double lastChosenTurretAngle = Double.NaN;

    public int alliance;
    public boolean far;

    private ServoImplEx turret1;
    private ServoImplEx turret2;

    public static double turretOffset = -4;
    public static double turretOffset2 = 0;
    public static double turretOffsetStep = -5;
    // Inches from the Pinpoint/Pedro robot pose origin to the turret pivot.
    public static double turretForwardOffset = -0.52588;
    public static double turretStrafeOffset = 0;

    public static double openStopperPos = 0;
    public static double closeStopperPos = 0.045;
    public Command driveToGate = new LambdaCommand()
            .setStart(() -> dToGate = true);
    public static boolean dToGate = false;
    // Loop time tracking
    private long lastLoopTime = 0;
    private double loopTimeMs = 0;
    public Pose currPose;

    // OPTIMIZATION: Cache LynxModules to avoid allocation every loop
    private List<LynxModule> allHubs;
    // OPTIMIZATION: Reduce telemetry frequency
    private int telemetryCounter = 0;
    private static final int TELEMETRY_EVERY_N_LOOPS = 5;
    // OPTIMIZATION: Avoid redundant servo writes
    private double lastServoPos = -1;
    private double lastAppliedOffset = Double.NaN;
    private final ElapsedTime slewTimer = new ElapsedTime();
    private double lastCmdAngle = Double.NaN;
    private long lastCmdNanos = 0;
    private double cmdRate = 0;
    private double lastLoggedServoSignal = 0;
    private double lastHeadingError = 0;
    private double lastHoodAngle = 0;

    public Supplier<Double> yVCtx;

    @Override
    public Command getDefaultCommand() {

        configureAllianceTarget();
        follower.update();
        currPose = follower.getPose();
        {
            return new MecanumDriverControlled(
                    fL,
                    fR,
                    bL,
                    bR,
                    Gamepads.gamepad1().leftStickY().map(it -> alliance * it),
                    Gamepads.gamepad1().leftStickX().map(it ->-1* alliance * it),
                    Gamepads.gamepad1().rightStickX().map(it -> rotationCommand(it)),
                    new FieldCentric(() -> Angle.fromRad(follower.getHeading() ))
            );
        }

        //return null;
    }

    public static MotorEx intakeMotor;
    public static MotorEx transfer;


    public Command localize;

    public static ServoEx hoodServo;   // constructed in initialize() - class-load init goes stale on hot runs



    private static final double MIN_ANGLE = -224.75;
    private static final double MAX_ANGLE = 224.75;

    // Tracks where the turret is across frames (Double, initialized to center)
    private double currentTurretPos = 0;
    public boolean wrapping = false;
    public static ServoEx stopperServo;   // constructed in initialize() - same static-hardware bug class

    /**
     * Pick a reachable turret angle for the requested bearing.
     *
     * The turret travels +/-224.75 deg, so bearings in the overlap band have
     * TWO reachable representations: 200 deg is reachable directly OR as -160.
     * The old version normalised to +/-180 and clamped, which threw away the
     * extra 44.75 deg of travel on each side AND forced a 360 deg jump every
     * time the command crossed 180. That jump is a step input into an
     * unbalanced counterroller load - the current spike that trips the Axon
     * protection and costs it its zero.
     *
     * This keeps whichever representation is nearest to where the turret
     * already is. NOTE: there is no hysteresis here - if a bearing sits right
     * between two representations, noise can still flip the choice. In the
     * overlap band that should not arise, because the nearest representation
     * is the one the turret is already on. Watch the 'wrapping' CSV column;
     * if wraps happen often in normal play, hysteresis is worth revisiting.
     */
    public double getClosestValidTurretAngle(double relativeGoalDegrees) {
        double base = normalizeDegrees(relativeGoalDegrees);

        double current = Double.isNaN(slewedTurretAngle) ? 0.0 : slewedTurretAngle;

        double best = Double.NaN;
        double bestCost = Double.MAX_VALUE;
        for (int k = -1; k <= 1; k++) {
            double cand = base + 360.0 * k;
            if (cand < MIN_ANGLE || cand > MAX_ANGLE) continue;
            double cost = Math.abs(cand - current);
            if (cost < bestCost) {
                bestCost = cost;
                best = cand;
            }
        }
        if (Double.isNaN(best)) {
            wrapping = false;
            lastChosenTurretAngle = Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, base));
            return lastChosenTurretAngle;
        }

        wrapping = Math.abs(best - current) > 90.0;
        lastChosenTurretAngle = best;
        return best;
    }

    /**
     * Slew-limit the commanded turret angle (Meta Infinity's structure: one
     * path, an ElapsedTime that is reset every call, clamp the delta to
     * rate * elapsed). The only addition is the rate being chosen by whether
     * a wrap is in progress - a wrap is a large sweep where one servo fights
     * the antagonistic preload, so it gets its own limit.
     *
     * @param target where the turret wants to be, degrees
     * @return the slew-limited command for this loop
     */
    private double slewTurret(double target) {
        double elapsedSec = slewTimer.seconds();
        slewTimer.reset();
        // First call, or a stall long enough that the elapsed time is
        // meaningless - do not let one huge dt authorise an unlimited jump.
        if (elapsedSec <= 0 || elapsedSec > 0.5) elapsedSec = 0.02;

        if (Double.isNaN(slewedTurretAngle)) slewedTurretAngle = target;

        double rate = wrapping ? maxSlewWrapDegPerSec : maxSlewNormalDegPerSec;
        double maxDelta = rate * elapsedSec;
        slewedTurretAngle += Range.clip(target - slewedTurretAngle, -maxDelta, maxDelta);
        return slewedTurretAngle;
    }

    /** Normalise to [-180, 180]. IEEEremainder does this in one call - no
     *  loop, and no chance of spinning on a NaN or huge input. */
    private double normalizeDegrees(double degrees) {
        return Math.IEEEremainder(degrees, 360.0);
    }

    private Pose getTurretPose(Pose robotPose) {
        double heading = robotPose.getHeading();

        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        double turretX = robotPose.getX()
                + turretForwardOffset * cos
                - turretStrafeOffset * sin;

        double turretY = robotPose.getY()
                + turretForwardOffset * sin
                + turretStrafeOffset * cos;

        return new Pose(turretX, turretY, heading);
    }

    private Vector getTurretToGoalVector(Pose turretPose) {
        return new Vector(
                turretPose.distanceFrom(new Pose(goalX, goalY)),
                Math.atan2(goalY - turretPose.getY(), goalX - turretPose.getX())
        );
    }

    /**
     * Rotation axis for the drive command. Normally the scaled right stick;
     * while gamepad1's left bumper is held, a proportional + static-feedforward
     * controller that turns the chassis to a fixed field heading instead.
     *
     * Releasing the bumper hands control straight back to the stick, so this
     * is also its own abort: if it ever runs away, let go.
     */
    private double rotationCommand(double stickX) {
        double stick = 0.8 * stickX;

        if (!headingLockEnabled || !headingLockHeld) {
            headingLockActive = false;
            return stick;
        }
        headingLockActive = true;

        double targetDeg = (alliance == -1) ? headingLockRedDeg : headingLockBlueDeg;
        double currentDeg = Math.toDegrees(follower.getPose().getHeading());

        // IEEEremainder wraps the error into [-180, 180], so the robot always
        // turns the short way round.
        double error = Math.IEEEremainder(currentDeg - targetDeg, 360.0);
        if (Double.isNaN(error)) {
            headingLockActive = false;
            return stick;
        }
        if (Math.abs(error) < headingLockDeadbandDeg) return 0.0;

        double outPower = error * headingKp + headingKs * Math.signum(error);
        outPower *= headingLockSign;
        if (outPower > headingLockMaxPower) outPower = headingLockMaxPower;
        if (outPower < -headingLockMaxPower) outPower = -headingLockMaxPower;
        return outPower;
    }

    private void configureAllianceTarget() {
        if (isRed()) {
            alliance = -1;
            goalXDist = 140;
            goalX = 140;
        } else if (isBlue()) {
            alliance = 1;
            goalXDist = 3;
            goalX = 3;
        } else {
            ActiveOpMode.telemetry().addLine("No direction set");
        }
    }



    public static Command closeStopper = new LambdaCommand()
            .setStart(() -> {
                stopperServo.setPosition(closeStopperPos); // close
            }).setIsDone(() -> true);
    public static Command openStopper = new LambdaCommand()
            .setStart(() -> {
                stopperServo.setPosition(openStopperPos); // open
            }).setIsDone(() -> true);


    @Override
    public void initialize() {

        firsttime = true;
        headingLockHeld = false;
        headingLockActive = false;
        slewedTurretAngle = Double.NaN;
        lastChosenTurretAngle = Double.NaN;
        slewTimer.reset();
        lastAppliedOffset = Double.NaN;

        shooting = false;
        follower = PedroComponent.follower();
        intakeMotor = new MotorEx("intakeMotor");
        transfer = new MotorEx("transferMotor");
        fL = new MotorEx("frontLeft");
        fR = new MotorEx("frontRight");
        bL = new MotorEx("backLeft");
        bR = new MotorEx("backRight");
        ShooterCalcAccelClaude.revAmpedMode=1;
        hoodServo = new ServoEx("hoodServo");
        stopperServo = new ServoEx("stopperServo");

        // OPTIMIZATION: Cache LynxModules once during initialization
        allHubs = ActiveOpMode.hardwareMap().getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        closeStopper.schedule();
        configureAllianceTarget();
        imu = new IMUEx("imu", Direction.LEFT, Direction.BACKWARD).zeroed();
        Pose startingpose = new Pose(72, 72, Math.toRadians(90));
        if(Storage.setPose){
            startingpose = Storage.currentPose;
            Storage.setPose=false;
        }
        follower.setStartingPose(startingpose);
        follower.setHeading(startingpose.getHeading());

        if (alliance == -1) {
            localize = new LambdaCommand()
                    .setStart(() -> follower.setPose(new Pose(128, 86, Math.toRadians(90))));

        }
        if (alliance == 1) {
            localize = new LambdaCommand()
                    .setStart(() -> follower.setPose(new Pose(16, 86, Math.toRadians(90))));

        }
        turret1 = ActiveOpMode.hardwareMap().get(ServoImplEx.class, "turretServo1");
        turret2 = ActiveOpMode.hardwareMap().get(ServoImplEx.class,"turretServo2");
        turret1.setPwmRange(new PwmControl.PwmRange(500, 2500));
        turret2.setPwmRange(new PwmControl.PwmRange(500, 2500));
        follower.update();
        /*try {
            if (!logAccel) throw new IOException("logging disabled");
            writer = new FileWriter("/sdcard/FIRST/accelLog.csv");
            t0 = System.nanoTime();
            writer.write("t,speed,aRaw,aFilt,omega,loopMs,headingError,cmdRate,targetTurret,servoSignal,lastServo,shooting,projSpeed,flightTime,reqTPS,hood,poseX,poseY,poseH,volts,wrapping\n");
        } catch (IOException e) {
            writer = null;   // log to telemetry only; opmode still drives
        }
        ActiveOpMode.telemetry().addData("logger", writer != null ? "ready" : "FILE FAILED");
        ActiveOpMode.telemetry().update();*/

    }
    private boolean autoShoot = true;

    public Command toggleAutoShoot = new LambdaCommand()
            .setStart(()->{
                if(autoShoot){
                    autoShoot = false;
                }
                else{
                    autoShoot = true;
                }
            });


    double goalY = 140.5;
    double goalX = 140;

    double goalXDist = 140;


    static boolean shooting = false;
    Command dToGateFalse = new LambdaCommand()
            .setStart(() -> dToGate = false);

    static Command shootFalse = new LambdaCommand()
            .setStart(() -> shooting = false);

    Command shooterer = new LambdaCommand()
            .setStart(() -> shoot());
    private static Command intakeMotorOn = new LambdaCommand()
            .setStart(() -> {
                intakeMotor.setPower(1);
                transfer.setPower(1);
            });


    private static Command intakeMotorOff = new LambdaCommand()
            .setStart(() -> {
                intakeMotor.setPower(0);
                transfer.setPower(0);
            });

    public boolean autoshoot = false;

    private FileWriter writer;
    private long t0;
    private int linesSinceFlush = 0;
    private long lastLoopNanos = 0;

    public static void shoot() {
        if (shooting == false) {
            shooting = true;
            SequentialGroup shoot = new SequentialGroup(openStopper, intakeMotorOn, new Delay(0.3), intakeMotorOff, closeStopper, shootFalse);
            shoot.schedule();
        }
    }

    public static void shootreal(){
        //shoot = true;
    }
    private double targetTurretAngle;
    private double avgLoopTime = 0;


    @Override
    public void periodic() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
        follower.update();


        double t = (System.nanoTime() - t0) / 1e9;
        double speed = PedroComponent.follower().getVelocity().getMagnitude();
        double aRaw = ShooterCalcAccelClaude.lastAlongTrackAccelRaw;
        double aFilt = ShooterCalcAccelClaude.lastAlongTrackAccel;
        double omega = PedroComponent.follower().getAngularVelocity();

        long now = System.nanoTime();
        double loopMs = lastLoopNanos == 0 ? 0 : (now - lastLoopNanos) / 1e6;
        lastLoopNanos = now;


        /*if (logAccel && writer != null) {
            try {
                double vbat = 12.0;
                if (!allHubs.isEmpty()) {
                    vbat = allHubs.get(0).getInputVoltage(VoltageUnit.VOLTS);
                }
                writer.write(String.format(
                        "%.4f,%.2f,%.1f,%.1f,%.4f,%.1f,%.2f,%.1f,%.2f,%.4f,%.4f,%d,%.2f,%.4f,%.0f,%.4f,%.2f,%.2f,%.4f,%.2f,%d%n",
                        t, speed, aRaw, aFilt, omega, loopMs,
                        lastHeadingError, cmdRate, targetTurretAngle,
                        lastLoggedServoSignal, lastServoPos,
                        shooting ? 1 : 0,
                        ShooterCalcAccelClaude.lastProjectedSpeed,
                        ShooterCalcAccelClaude.lastFlightTime,
                        ShooterCalcAccelClaude.requiredTPS,
                        lastHoodAngle,
                        follower.getPose().getX(), follower.getPose().getY(),
                        follower.getPose().getHeading(),
                        vbat, wrapping ? 1 : 0));
                if (++linesSinceFlush >= 20) {   // survive a crash or E-stop
                    writer.flush();
                    linesSinceFlush = 0;
                }
            } catch (IOException ignored) { }
        }*/

        ActiveOpMode.telemetry().addData("t", "%.1f s", t);
        ActiveOpMode.telemetry().addData("speed", "%.1f", speed);
        ActiveOpMode.telemetry().addData("aRaw", "%.0f", aRaw);
        ActiveOpMode.telemetry().addData("aFilt", "%.0f", aFilt);

        long currentTime = System.nanoTime();
        if (lastLoopTime != 0) {
            loopTimeMs = (currentTime - lastLoopTime) / 1_000_000.0;
            avgLoopTime +=loopTimeMs;
        }
        lastLoopTime = currentTime;


        if (firsttime == true) {
            // Schedule the command stored in the localize variable
            Gamepads.gamepad1().rightBumper().whenBecomesTrue(()->openStopper.schedule())
                    .whenBecomesFalse(()->closeStopper.schedule());
            Gamepads.gamepad1().rightTrigger().greaterThan(0.3).whenBecomesTrue(shooterer);
            // Heading lock is HELD, not toggled - same shape as the stopper
            // binding above, just setting a flag instead of scheduling.
            Gamepads.gamepad1().leftBumper()
                    .whenBecomesTrue(() -> { headingLockHeld = true; })
                    .whenBecomesFalse(() -> { headingLockHeld = false; });

            // RELOCALIZE. This MUST stay inside the firsttime guard.
            //
            // It used to sit below, outside the guard, so it re-registered on
            // EVERY periodic() call - roughly 40 new bindings per second. They
            // all stayed attached to the same button, so pressing X fired every
            // one of them at once: thousands of follower.setPose() calls, each
            // an I2C write to the Pinpoint, in a single loop iteration. That is
            // the multi-second freeze with dead inputs and unpowered motors.
            // It also got worse the later in the match you pressed it, because
            // more had piled up.
            //
            // One binding now, with the alliance read when it FIRES rather than
            // when it is registered.
            Gamepads.gamepad1().x().whenBecomesTrue(() -> {
                Pose relocalizePose = (alliance == 1)
                        ? new Pose(14.6, 77.5, Math.toRadians(90))
                        : new Pose(127.4, 77.5, Math.toRadians(90));
                follower.setPose(relocalizePose);
            });

            firsttime = false;
        }
        //follower.update();


        Pose currPose = follower.getPose();
        Vector velocity = follower.getVelocity();


        Pose turretPose = getTurretPose(currPose);
        double robotHeading = currPose.getHeading();
        Vector robotToGoalVector = getTurretToGoalVector(turretPose);

        double robotAngularVelocityRads = follower.getAngularVelocity();
        if (Double.isNaN(robotAngularVelocityRads)) robotAngularVelocityRads = 0.0;

        Double[] results = calculateShotVectorandUpdateHeading(
                robotHeading,
                robotToGoalVector,
                velocity,
                follower.getAcceleration()
        );
        double headingError = results[2];
        lastHeadingError = headingError;
        double flywheelSpeed = results[0];

        if (!Double.isNaN(flywheelSpeed)) {
            shooter((float) flywheelSpeed);
            currentMotorSpeed = flywheelSpeed;
        }
        double hoodAngle = results[1];
        lastHoodAngle = hoodAngle;
        if (!Double.isNaN(hoodAngle)) {
            hoodServo.setPosition(Math.max(0.05, Math.min(1.0, hoodAngle)));
        }

        double robotAngularVelocityDegs = Math.toDegrees(robotAngularVelocityRads);
        if (Double.isNaN(robotAngularVelocityDegs)) robotAngularVelocityDegs = 0.0;
        double feedforwardOffset = robotAngularVelocityDegs * yawFeedforwardGain;

        double allianceOffset = (alliance == -1) ? turretOffset : turretOffset2;

        // MEASUREMENT ONLY - cmdRate is logged, not applied. If the turret is
        // lagging a ramping command during braking, it will show up as a large
        // sustained cmdRate with servoSignal trailing targetTurret.
        /*double cmdForRate = headingError + allianceOffset;
        if (!Double.isNaN(cmdForRate)) {
            long cmdNanos = System.nanoTime();
            if (!Double.isNaN(lastCmdAngle) && lastCmdNanos != 0) {
                double cdt = (cmdNanos - lastCmdNanos) / 1e9;
                double r = (cmdForRate - lastCmdAngle) / Math.max(cdt, 1e-4);
                if (Math.abs(r) < cmdRateSpikeLimit) {
                    cmdRate += cmdRateFilterAlpha * (r - cmdRate);
                }
            }
            lastCmdAngle = cmdForRate;
            lastCmdNanos = cmdNanos;
        }*/

        double commandedAngle = headingError + allianceOffset - feedforwardOffset;

        if (turretParked) {
            // Parked: hold mechanical centre, no tracking. Slew-limited so entering park is not abrupt.
            double parkAngle = MIN_ANGLE + ((turretParkSignal - 0.05) / 0.90) * 449.51;
            targetTurretAngle = slewTurret(parkAngle);
            lastChosenTurretAngle = targetTurretAngle;

            double parkSignal =
                    0.05 + ((slewedTurretAngle - MIN_ANGLE) / 449.51) * 0.90;
            parkSignal = Math.max(0.05, Math.min(0.95, parkSignal));

            //No offset so can rezero
            turret1.setPosition(parkSignal);
            turret2.setPosition(parkSignal);
            lastServoPos = parkSignal;
            lastLoggedServoSignal = parkSignal;
            currentTurretPos = slewedTurretAngle;
            cmdRate = 0;
        } else if (!Double.isNaN(commandedAngle)) {
            double rawTarget = getClosestValidTurretAngle(commandedAngle);
            targetTurretAngle = slewTurret(rawTarget);

            double servoPositionSignal =
                    0.05 + ((targetTurretAngle - MIN_ANGLE) / 449.51) * 0.90;
            servoPositionSignal = Math.max(0.05, Math.min(0.95, servoPositionSignal));

            // No fighting when wrap
            double activeOffset = wrapping ? wrapServoOffset : servoOffset;

            // Only write when the command actually moved or if servoOffset was changed
            boolean offsetChanged = Math.abs(activeOffset - lastAppliedOffset) > 1e-9
                    || Double.isNaN(lastAppliedOffset);
            if (lastServoPos < 0
                    || offsetChanged
                    || Math.abs(servoPositionSignal - lastServoPos) > turretServoThreshold) {
                double finalServoPositionSignal = servoPositionSignal;
                Command turretServo1Update = new LambdaCommand()
                        .setStart(() -> {
                            turret1.setPosition(finalServoPositionSignal + activeOffset);
                        });
                Command turretServo2Update = new LambdaCommand()
                        .setStart(() -> {
                            turret2.setPosition(finalServoPositionSignal - activeOffset);
                        });
                ParallelGroup setTurret  = new ParallelGroup(turretServo1Update, turretServo2Update);
                setTurret.schedule();
                //turret1.setPosition(servoPositionSignal + activeOffset);
                //turret2.setPosition(servoPositionSignal - activeOffset);
                lastServoPos = servoPositionSignal;
                lastAppliedOffset = activeOffset;
            }
            lastLoggedServoSignal = servoPositionSignal;
            currentTurretPos = targetTurretAngle;

            ActiveOpMode.telemetry().addData("turret", servoPositionSignal);
            ActiveOpMode.telemetry().addData("turretParked", turretParked);
        }


        ActiveOpMode.telemetry().addData("hoodAngle", hoodAngle);
        ActiveOpMode.telemetry().addData("ballVelocity", flywheelSpeed);
        ActiveOpMode.telemetry().addData("flywheelSpeed", requiredTPS);
        ActiveOpMode.telemetry().addData("launch?", isOverlappingLaunchZone(currPose));
        ActiveOpMode.telemetry().addData("Loop Time (ms)", loopTimeMs);
        ActiveOpMode.telemetry().addData("Avg Loop Time", avgLoopTime);
        ActiveOpMode.telemetry().addData("alliance", alliance);
        ActiveOpMode.telemetry().addData("goalX", goalX);
        ActiveOpMode.telemetry().addData("goalY", goalY);
        ActiveOpMode.telemetry().addData("RobotX", currPose.getX());
        ActiveOpMode.telemetry().addData("RobotY", currPose.getY());
        ActiveOpMode.telemetry().addData("Robot Heading: ", follower.getHeading());
        ActiveOpMode.telemetry().addData("Turret Offset", turretOffset);
        ActiveOpMode.telemetry().addData("headingLock", headingLockActive);
        ActiveOpMode.telemetry().update();
    }

    public void onStop() {
        /*try {
            if (writer != null) { writer.flush(); writer.close(); }
        } catch (IOException ignored) { }*/
    }


    // Fixed constructor name to match class name exactly
   /* public Command getDriveToGateCommand() {
        return new LambdaCommand()
                .setStart(() -> {
                    // 1. Build the path dynamically from your current position
                    PathChain dGatePath = follower.pathBuilder()
                            .addPath(
                                    new BezierCurve(
                                            follower.getPose(),
                                            new Pose(115.158, 61.289),
                                            new Pose(133, 55.5)
                                    )
                            )
                            .setLinearHeadingInterpolation(follower.getPose().getHeading(), Math.toRadians(32))
                            .setVelocityConstraint(1.0)
                            .setTValueConstraint(0.8)

                            // 2. FIXED: Loosen the arrival tolerances directly on this specific path segment
                            // This allows Pedro to mark the path complete without infinitely trying to correct micro-inches
                            .setTranslationalConstraint(0.5) // Allow 1.5 inches of margin at destination
                            .setHeadingConstraint(Math.toRadians(2.0)) // Allow 3 degrees of heading margin
                            .build();

                    // 3. Command Pedro to run the path
                    follower.followPath(dGatePath, false);
                })
                // 4. End the command when Pedro stops driving
                .setIsDone(() -> !follower.isBusy())
                .setStop((Boolean interrupted) -> {
                    follower.breakFollowing(); // Safely hand control back to the joysticks
                })
                .requires(this); // Lock out your TeleOp joysticks while this is executing
    }*/
}