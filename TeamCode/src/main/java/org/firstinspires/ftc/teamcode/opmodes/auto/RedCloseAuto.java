package org.firstinspires.ftc.teamcode.opmodes.auto;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static org.firstinspires.ftc.teamcode.hdware.final_Globals.AUTO_SHOT_TIMEOUT_S;
import static org.firstinspires.ftc.teamcode.hdware.final_Globals.AUTO_TURRET_FLOWER_POS;
import static org.firstinspires.ftc.teamcode.hdware.final_Globals.AUTO_TURRET_POS;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.pedro.PedroCommands;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hdware.FinalHardware;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.sequences.FinalSequences2;
import org.firstinspires.ftc.teamcode.subsystems.FinalIntake2Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.FinalSensorThread2;
import org.firstinspires.ftc.teamcode.subsystems.FinalShooter2Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.FinalShooter2Subsystem.FLAP;
import org.firstinspires.ftc.teamcode.subsystems.FinalShooter2Subsystem.MODE;
import org.firstinspires.ftc.teamcode.subsystems.FinalShooter2Subsystem.SERVO2;

/*
 * ============================== RED CLOSE AUTO ==============================
 *
 * Paths from the Pedro visualizer (Pedro 3.0 export) + robot sequences around them.
 * You only edit sections 1, 2 and 3. Everything below "ROBOT SETUP" stays the same.
 *
 *   1. POSES + PATHS  - paste from the visualizer export
 *   2. ROUTINE        - the order of everything (paths + sequences)
 *   3. MY SEQUENCES   - your own named sequences, so they are easy to swap
 *
 * Building blocks for the routine (section 2):
 *
 *   BEFORE a path   put the sequence on the line before:   a.intakeOn(), path(gardenIntake()),
 *   AFTER a path    put the sequence on the line after:    path(gardenIntake()), a.shoot(),
 *   DURING a path   pass it to path(...):                  path(flowerIntake(), a.spinUp()),
 *     at a point    at(0.5, cmd)  -> starts when the path is 50 % done
 *     after a time  after(1.2, cmd) -> starts 1.2 s after the path started
 *   wait            pause(0.5)
 *   at the same time  together(cmdA, cmdB)
 *
 *   path(...) ends when the robot has STOPPED at the end of the path AND all its
 *   "during" sequences are finished.
 *
 * Ready-made sequences (AutoActions a): a.intakeOn(), a.intakeOff(), a.spinUp(), a.shooterOff(),
 *   a.waitForSpeed(), a.shoot(), a.stopAll(), a.holdHere()
 */
@Autonomous(name = "Red Close Auto", group = "Final", preselectTeleOp = "Final Command TeleOp 2 (Ivy)")
public class RedCloseAuto extends LinearOpMode {

    // =====================================================================================
    // 1. POSES + PATHS  (paste from the visualizer export)
    // =====================================================================================

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56.3594, 8.8984, 180);
    private final Pose start_2 = poseFactory.of(56.1467, 9.4148, 180);
    private final Pose gardenIntake = poseFactory.of(13.9543, 9.407, 180);
    private final Pose flowerIntakeStart = poseFactory.of(13.9543, 9.407, 180);
    private final Pose flowerIntake = poseFactory.of(47.684, 126.6051, 90);
    private final Pose flowerIntakeControl1 = poseFactory.of(19.6785, 89.1304, 0);
    private final Pose parkStart = poseFactory.of(47.684, 126.6051, 90);
    private final Pose park = poseFactory.of(9.3408, 109.4643, 90);

    /** Robot is placed here at init. */
    private Pose startPose() {
        return start;
    }

    public Path startPath() {
        return Paths.line(start, start_2).constant(start_2);
    }

    public Path gardenIntake() {
        return Paths.line(start, gardenIntake).constant(gardenIntake);
    }

    public Path flowerIntake() {
        return Paths.curve(flowerIntakeStart, flowerIntakeControl1, flowerIntake).linear(flowerIntakeStart, flowerIntake);
    }

    public Path park() {
        return Paths.line(parkStart, park).linear(parkStart, park);
    }

    // =====================================================================================
    // 2. ROUTINE  - read it top to bottom, it runs in this order
    // =====================================================================================

    private Command routine() {
        return sequential(
                // ---------- 1. Path 1 + shoot the PRELOAD ----------
                step("path 1"),
                path(startPath(),
                        a.spinUp()),                       // DURING: flywheel spins up while driving
                shootPollen("preload", TURRET_PRELOAD),    // AFTER: shoot preload

                // ---------- 2. Garden intake ----------
                step("garden intake"),
                path(gardenIntake(),
                        after(0.1, a.intakeOn())),         // DURING: intake on (just after leaving)
                keepIntaking(GARDEN_INTAKE_EXTRA_S),       // AFTER: keep intaking a moment

                // ---------- 3. Drive to flower position, shoot garden pollen ----------
                step("to flower"),
                path(flowerIntake(),
                        at(0.2, a.intakeOff()),            // DURING: intake off at 20 %
                        at(0.5, servo2Down()),             //         servo2 down at 30 %
                        at(0.5, a.spinUp())),              //         flywheel on at 60 %
                shootPollen("garden pollen", TURRET_FLOWER),

                // ---------- 3b. Flower intake ----------
                flowerIntakeSequence(),

                // ---------- 4. Shoot the flower pollen ----------
                shootPollen("flower pollen", TURRET_FLOWER),

                // ---------- 5. Park ----------
                step("park"),
                path(park(),
                        a.stopAll(),                       // DURING: shooter / intake off, flap closed
                        servo2Up())                        //         servo2 back up
        );
    }

    // =====================================================================================
    // 3. MY SEQUENCES  - make your own here, then use them in the routine
    // =====================================================================================

    // ---- settings (seconds / servo positions) ----
    /** Turret position when shooting the preload (turret perpendicular, facing the hive). */
    public static double TURRET_PRELOAD = AUTO_TURRET_POS;
    /** Turret position when shooting at the flower position (robot heading is different there!). */
    public static double TURRET_FLOWER = AUTO_TURRET_FLOWER_POS;
    /** Wait after opening the flap before feeding. */
    public static double FLAP_OPEN_WAIT_S = 0.15;
    /** Keep the intake running this long after the garden path ends. */
    public static double GARDEN_INTAKE_EXTRA_S = 0.2;
    /** Wait after servo2 goes down before the intake starts. */
    public static double SERVO2_DOWN_WAIT_S = 0.3;
    /** Flower intake runs until the robot is full, or at most this long. */
    public static double FLOWER_INTAKE_MAX_S = 2.5;

    /**
     * Shoot all pollen:  turret -> POLLEN mode -> flywheel at speed -> FLAP OPEN -> feed until
     * the shot ends -> FLAP CLOSE.  The flywheel stays on for the next shot.
     */
    private Command shootPollen(String name, double turretPos) {
        return sequential(
                step("shoot " + name),
                turret(turretPos),
                shooterSS.forceModeCommand(MODE.POLLEN),
                a.spinUp(),
                a.waitForSpeed(),
                flapOpen(),
                pause(FLAP_OPEN_WAIT_S),
                FinalSequences2.shootStart(intakeSS, shooterSS, sensors),   // KICKBACK -> WAIT -> FEEDING
                waitUntil(() -> !intakeSS.isShooting())                     // ends by itself after SHOOT_FLAP_CLOSE_S
                        .raceWith(pause(AUTO_SHOT_TIMEOUT_S)),
                FinalSequences2.stopShooting(intakeSS, shooterSS),          // feeder + intake off
                flapClose()
        );
    }

    /** At the flower position: servo2 down (intake position), intake on until full, intake off. */
    private Command flowerIntakeSequence() {
        return sequential(
                step("flower intake"),
                servo2Down(),                               // already down from the path, makes sure
                pause(SERVO2_DOWN_WAIT_S),
                a.intakeOn(),                               // also closes the flap + resets the pollen count
                waitUntil(intakeSS.pollen::isFull)
                        .raceWith(pause(FLOWER_INTAKE_MAX_S)),
                a.intakeOff()
        );
    }

    /** Keep intaking for some seconds (intake already on), then off. */
    private Command keepIntaking(double seconds) {
        return sequential(pause(seconds), a.intakeOff());
    }

    // ---- single parts ----
    private Command flapOpen()   { return shooterSS.flapCommand(FLAP.OPEN); }
    private Command flapClose()  { return shooterSS.flapCommand(FLAP.CLOSE); }
    private Command servo2Down() { return shooterSS.servo2Command(SERVO2.OPEN); }   // dpad down in TeleOp
    private Command servo2Up()   { return shooterSS.servo2Command(SERVO2.CLOSE); }  // dpad up in TeleOp
    private Command turret(double pos) { return instant(() -> robot.setTurretPosition(pos)); }

    // =====================================================================================
    // BUILDING BLOCKS  (no need to change)
    // =====================================================================================

    /** Robot must be stopped at the end of a path within this time, or the routine moves on. */
    public static double SETTLE_TIMEOUT_S = 1.0;
    /** A path is cut off after this long (robot stuck). */
    public static double PATH_TIMEOUT_S = 8.0;

    /**
     * Drive a path. Any commands given start together with the path ("during").
     * Ends when the robot has stopped at the end AND the during-commands are done.
     */
    private Command path(Path p, Command... during) {
        Command drive = sequential(
                instant(() -> pathStartLoop = loopCount),
                PedroCommands.follow(follower, p).raceWith(waitMs(PATH_TIMEOUT_S * 1000)),
                // follow() ends when the robot passes the end, still moving. Let it stop there.
                waitUntil(() -> !follower.isBusy()).raceWith(waitMs(SETTLE_TIMEOUT_S * 1000))
        );
        if (during.length == 0) return drive;
        Command[] all = new Command[during.length + 1];
        all[0] = drive;
        System.arraycopy(during, 0, all, 1, during.length);
        return parallel(all);
    }

    /** Inside path(...): start cmd when the path is this far done (0.0 - 1.0). */
    private Command at(double fraction, Command cmd) {
        return sequential(
                // loopCount check: wait until the follower has updated on the NEW path at least once
                waitUntil(() -> loopCount > pathStartLoop && follower.completion() >= fraction),
                cmd
        );
    }

    /** Inside path(...): start cmd this many seconds after the path started. */
    private Command after(double seconds, Command cmd) {
        return sequential(pause(seconds), cmd);
    }

    /** Run commands at the same time; ends when all are done. */
    private Command together(Command... cmds) {
        return parallel(cmds);
    }

    /** Wait this many seconds. */
    private Command pause(double seconds) {
        return waitMs(seconds * 1000);
    }

    /** Name shown on telemetry as "Step". */
    private Command step(String name) {
        return a.label(name);
    }

    // =====================================================================================
    // ROBOT SETUP  (same as BaseAuto / FinalCommandTeleOp2)
    // =====================================================================================

    private final FinalHardware robot = FinalHardware.getInstance();
    private Follower follower;
    private FinalShooter2Subsystem shooterSS;
    private FinalIntake2Subsystem intakeSS;
    private FinalSensorThread2 sensors;
    private AutoActions a;

    private volatile int loopCount = 0;
    private volatile int pathStartLoop = 0;

    private final ElapsedTime matchTimer = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        Scheduler.reset();

        robot.init(hardwareMap);
        follower = Constants.create(hardwareMap);
        shooterSS = new FinalShooter2Subsystem(robot);
        intakeSS = new FinalIntake2Subsystem(robot, shooterSS);
        sensors = new FinalSensorThread2(robot, intakeSS, shooterSS, gamepad1);
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        a = new AutoActions(follower, intakeSS, shooterSS, sensors);

        follower.setPose(startPose());
        follower.update();

        Command auto = sequential(
                FinalSequences2.initSequence(intakeSS, shooterSS),
                routine(),
                a.label("done"),
                a.stopAll()
        );

        while (opModeInInit()) {
            telemetry.addData("PLACE ROBOT AT", fmt(startPose()));
            telemetry.addData("B1 / limit 1 / limit 2", "%s / %s / %s",
                    robot.isB1Broken(), robot.isLimit1Pressed(), robot.isLimit2Pressed());
            telemetry.addLine("Ready - press START");
            telemetry.update();
        }
        if (isStopRequested()) return;

        try {
            robot.setTurretPosition(TURRET_PRELOAD);
            sensors.start();
            matchTimer.reset();
            loopTimer.reset();
            Scheduler.schedule(auto);

            while (opModeIsActive()) {
                double loopMs = loopTimer.milliseconds();
                loopTimer.reset();

                Scheduler.execute();       // routine steps
                shooterSS.periodic();      // flywheel
                intakeSS.periodic();       // intake / feeder
                follower.update();         // drive
                loopCount++;

                showTelemetry(loopMs);
            }
        } finally {
            BaseAuto.lastPose = follower.pose();   // TeleOp can start from here
            sensors.stop();
            intakeSS.stop();
            shooterSS.stop();
            follower.stop();
            Scheduler.reset();
        }
    }

    private void showTelemetry(double loopMs) {
        telemetry.addData("Time", "%.1f s", matchTimer.seconds());
        telemetry.addData("Step", a.step);
        telemetry.addData("Pose", fmt(follower.pose()));
        telemetry.addData("Path done", "%.0f %%", follower.completion() * 100);
        telemetry.addData("SHOOTER", "%s %s  %.0f / %.0f", shooterSS.shooterState, shooterSS.modeState,
                shooterSS.velocity, shooterSS.target);
        telemetry.addData("SHOT / INTAKE", "%s / %s", intakeSS.shotState, intakeSS.intakeState);
        telemetry.addData("Pollen", intakeSS.pollen.getCount());
        telemetry.addData("Loop", "%.1f ms", loopMs);
        telemetry.update();
    }

    private static String fmt(Pose p) {
        return String.format("x %.1f  y %.1f  h %.0f deg", p.x(), p.y(), Math.toDegrees(p.heading()));
    }
}
