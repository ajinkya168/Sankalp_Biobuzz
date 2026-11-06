package org.firstinspires.ftc.teamcode.opmodes.auto;

import static com.pedropathing.ivy.groups.Groups.sequential;
import static org.firstinspires.ftc.teamcode.hdware.final_Globals.AUTO_TURRET_POS;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hdware.FinalHardware;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.visualizer.VisualizerPaths;
import org.firstinspires.ftc.teamcode.sequences.FinalSequences2;
import org.firstinspires.ftc.teamcode.subsystems.FinalIntake2Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.FinalSensorThread2;
import org.firstinspires.ftc.teamcode.subsystems.FinalShooter2Subsystem;

/*
 * ============================== BASE AUTO ==============================
 *
 * Everything an autonomous needs, set up the same way as FinalCommandTeleOp2:
 *   FinalHardware + Pedro follower + FinalShooter2 / FinalIntake2 / FinalSensorThread2,
 *   bulk reads, turret held at AUTO_TURRET_POS, telemetry.
 *
 * A real auto only says WHICH exported paths class to use and WHAT to do with the paths:
 *
 *   @Autonomous(name = "My Auto")
 *   public class MyAuto extends BaseAuto {
 *       protected Class<?> pathsClass() { return MyExportedPaths.class; }
 *       protected Command routine(VisualizerPaths p, AutoActions a) {
 *           return sequential(a.scoreAt("preload", p.get(0)), a.collect("row 1", p.get(1)), ...);
 *       }
 *   }
 *
 * The robot's start pose = start of the first exported path. Place the robot there.
 *
 * Every loop:  Scheduler.execute() -> shooter / intake periodic() -> turret -> follower.update()
 */
public abstract class BaseAuto extends LinearOpMode {

    /** Pose at the end of the last auto - a TeleOp can start from here. */
    public static Pose lastPose = null;

    protected final FinalHardware robot = FinalHardware.getInstance();
    protected Follower follower;
    protected FinalShooter2Subsystem shooterSS;
    protected FinalIntake2Subsystem intakeSS;
    protected FinalSensorThread2 sensors;
    protected AutoActions actions;
    protected VisualizerPaths paths;

    private final ElapsedTime matchTimer = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();

    /** The class exported by the Pedro visualizer (see opmodes/auto/README.md). */
    protected abstract Class<?> pathsClass();

    /** The whole autonomous as one command, built from the paths and the actions. */
    protected abstract Command routine(VisualizerPaths p, AutoActions a);

    @Override
    public final void runOpMode() {
        Scheduler.reset();

        // ------------------------------ hardware + subsystems ------------------------------
        robot.init(hardwareMap);
        follower = Constants.create(hardwareMap);
        shooterSS = new FinalShooter2Subsystem(robot);
        intakeSS = new FinalIntake2Subsystem(robot, shooterSS);
        sensors = new FinalSensorThread2(robot, intakeSS, shooterSS, gamepad1);
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        actions = new AutoActions(follower, intakeSS, shooterSS, sensors);

        // ------------------------------ paths ------------------------------
        try {
            paths = VisualizerPaths.load(pathsClass());
        } catch (RuntimeException e) {
            telemetry.addLine("PATHS FAILED TO LOAD - fix " + pathsClass().getSimpleName());
            telemetry.addLine(String.valueOf(e.getMessage()));
            telemetry.update();
            waitForStart();
            return;
        }
        follower.setPose(paths.start());
        Command routine = sequential(
                FinalSequences2.initSequence(intakeSS, shooterSS),
                routine(paths, actions),
                actions.label("done"),
                actions.stopAll()
        );

        // ------------------------------ init loop ------------------------------
        while (opModeInInit()) {
            telemetry.addData("Path1", "%s (%d)", pathsClass().getSimpleName(), paths.size());
            telemetry.addData("Order", paths.names());
            telemetry.addData("PLACE ROBOT AT", fmt(paths.start()));
            telemetry.addData("B1 / limit 1 / limit 2", "%s / %s / %s",
                    robot.isB1Broken(), robot.isLimit1Pressed(), robot.isLimit2Pressed());
            telemetry.addLine("Ready - press START");
            telemetry.update();
        }
        if (isStopRequested()) return;

        // ------------------------------ run ------------------------------
        try {
            robot.setTurretPosition(AUTO_TURRET_POS);
            sensors.start();
            matchTimer.reset();
            loopTimer.reset();
            Scheduler.schedule(routine);

            while (opModeIsActive()) {
                double loopMs = loopTimer.milliseconds();
                loopTimer.reset();

                Scheduler.execute();       // auto steps change states
                shooterSS.periodic();      // SHOOTER + MODE -> flywheel power
                intakeSS.periodic();       // INTAKE + SHOT  -> intake / feeder power
                follower.update();         // drive

                showTelemetry(loopMs);
            }
        } finally {
            lastPose = follower.pose();
            sensors.stop();
            intakeSS.stop();
            shooterSS.stop();
            follower.stop();
            Scheduler.reset();
        }
    }

    private void showTelemetry(double loopMs) {
        telemetry.addData("Time", "%.1f s", matchTimer.seconds());
        telemetry.addData("Step", actions.step);
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
