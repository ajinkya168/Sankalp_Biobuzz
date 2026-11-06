package org.firstinspires.ftc.teamcode.opmodes.auto;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.lazy;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static org.firstinspires.ftc.teamcode.hdware.final_Globals.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.pedro.PedroCommands;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.sequences.FinalSequences2;
import org.firstinspires.ftc.teamcode.subsystems.FinalIntake2Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.FinalIntake2Subsystem.INTAKE;
import org.firstinspires.ftc.teamcode.subsystems.FinalIntake2Subsystem.SHOT;
import org.firstinspires.ftc.teamcode.subsystems.FinalSensorThread2;
import org.firstinspires.ftc.teamcode.subsystems.FinalShooter2Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.FinalShooter2Subsystem.FLAP;
import org.firstinspires.ftc.teamcode.subsystems.FinalShooter2Subsystem.SHOOTER;

/*
 * ============================== AUTO ACTIONS (Ivy) ==============================
 *
 * The TeleOp's button actions (FinalSequences2) as autonomous steps. Where a TeleOp action is
 * "press and let it run", the auto version also WAITS until it is done, so steps can be put
 * one after the other in sequential(...).
 *
 *   TELEOP BUTTON      AUTO STEP
 *   A (intake on)      intakeOn()        - FinalSequences2.intakeStart
 *   dpad left          intakeOff()       - FinalSequences2.intakeStop
 *   B (shooter on)     spinUp()          - SHOOTER ON (velocity from POLLEN / NECTAR mode)
 *   LB (shoot)         shoot()           - spin up, wait for speed, FinalSequences2.shootStart,
 *                                          wait for the shot to end, stopShooting
 *   -                  follow(path)      - drive a path, ends at the end of the path
 *   -                  collect(path)     - intake on, drive, keep intaking AUTO_INTAKE_SETTLE_S
 *   -                  scoreAt(path)     - drive while spinning up, then shoot()
 *   -                  park(path)        - everything off, drive
 *   -                  stopAll()         - shooter, intake, shot off, flap closed
 *
 * Every method returns a NEW command, so call it again each time you use it.
 * Timeouts read final_Globals when the step starts, so FTC Dashboard changes apply.
 */
public class AutoActions {

    private final Follower follower;
    private final FinalIntake2Subsystem intakeSS;
    private final FinalShooter2Subsystem shooterSS;
    private final FinalSensorThread2 sensors;

    /** Name of the step running now, for telemetry. */
    public volatile String step = "-";

    public AutoActions(Follower follower, FinalIntake2Subsystem intakeSS,
                       FinalShooter2Subsystem shooterSS, FinalSensorThread2 sensors) {
        this.follower = follower;
        this.intakeSS = intakeSS;
        this.shooterSS = shooterSS;
        this.sensors = sensors;
    }

    // =========================== DRIVE ===========================

    /** Follow a path; ends at its end, or after AUTO_PATH_TIMEOUT_S if the robot gets stuck. */
    public Command follow(Path path) {
        return PedroCommands.follow(follower, path).raceWith(timeout(() -> AUTO_PATH_TIMEOUT_S));
    }

    /** Stay where the robot is now (pose read when this step starts). */
    public Command holdHere() {
        return instant(() -> follower.hold(follower.pose()));
    }

    // =========================== INTAKE ===========================

    public Command intakeOn() {
        return FinalSequences2.intakeStart(intakeSS, shooterSS);
    }

    public Command intakeOff() {
        return FinalSequences2.intakeStop(intakeSS, shooterSS);
    }

    // =========================== SHOOTER ===========================

    public Command spinUp() {
        return shooterSS.shooterCommand(SHOOTER.ON);
    }

    public Command shooterOff() {
        return shooterSS.shooterCommand(SHOOTER.OFF);
    }

    /** Wait until the flywheel is at speed (or AUTO_SPINUP_TIMEOUT_S passes). */
    public Command waitForSpeed() {
        return waitUntil(shooterSS::atSpeed).raceWith(timeout(() -> AUTO_SPINUP_TIMEOUT_S));
    }

    /**
     * LB in auto: spin up, wait for speed, start the shot (KICKBACK -> WAIT -> FEEDING), wait until
     * the shot ends by itself (SHOOT_FLAP_CLOSE_S) or AUTO_SHOT_TIMEOUT_S, then stop feeding + close
     * the flap. The shooter keeps spinning for the next shot.
     */
    public Command shoot() {
        return sequential(
                label("shoot: spin up"),
                spinUp(),
                waitForSpeed(),
                label("shoot: feeding"),
                FinalSequences2.shootStart(intakeSS, shooterSS, sensors),
                waitUntil(() -> !intakeSS.isShooting()).raceWith(timeout(() -> AUTO_SHOT_TIMEOUT_S)),
                FinalSequences2.stopShooting(intakeSS, shooterSS)
        );
    }

    // =========================== COMBINED STEPS ===========================

    /** Intake on, drive the path, keep intaking AUTO_INTAKE_SETTLE_S. The intake stays on afterwards. */
    public Command collect(String name, Path path) {
        return sequential(
                label("collect: " + name),
                intakeOn(),
                follow(path),
                timeout(() -> AUTO_INTAKE_SETTLE_S)
        );
    }

    /** Drive the path while the flywheel spins up, then shoot everything. */
    public Command scoreAt(String name, Path path) {
        return sequential(
                label("score: " + name),
                parallel(follow(path), spinUp()),
                shoot()
        );
    }

    /** Drive only - mechanisms keep doing whatever they were doing. */
    public Command drive(String name, Path path) {
        return sequential(label("drive: " + name), follow(path));
    }

    /** Everything off, then drive (end of auto). */
    public Command park(String name, Path path) {
        return sequential(label("park: " + name), stopAll(), follow(path));
    }

    /** Shooter off, intake off, shot ended, flap closed. */
    public Command stopAll() {
        return parallel(
                shooterSS.shooterCommand(SHOOTER.OFF),
                intakeSS.shotCommand(SHOT.IDLE),
                intakeSS.intakeCommand(INTAKE.OFF),
                shooterSS.flapCommand(FLAP.CLOSE)
        );
    }

    // =========================== HELPERS ===========================

    /** Wait a number of seconds. */
    public Command waitSeconds(double seconds) {
        return waitMs(seconds * 1000);
    }

    /** Sets the telemetry step name. */
    public Command label(String name) {
        return instant(() -> step = name);
    }

    // wait whose length is read from final_Globals when it starts
    private static Command timeout(java.util.function.DoubleSupplier seconds) {
        return lazy(() -> waitMs(seconds.getAsDouble() * 1000));
    }
}
