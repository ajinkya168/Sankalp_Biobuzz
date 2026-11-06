package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.hdware.final_Globals.*;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.hdware.FinalHardware;
import org.firstinspires.ftc.teamcode.subsystems.FinalShooter2Subsystem.FLAP;
import org.firstinspires.ftc.teamcode.subsystems.FinalShooter2Subsystem.MODE;

/*
 * ============================== SENSOR THREAD 2 ==============================
 *
 * Same as FinalSensorThread, for FinalIntake2Subsystem + FinalShooter2Subsystem
 * (FinalCommandTeleOp2). Polls b1 and both limit switches every WATCHER_SLEEP_MS on its
 * own thread, so they react right away instead of once per OpMode loop.
 *
 *   POLLEN COUNT (b1)  - one pollen = b1 broken >= MIN_BREAK_MS, then clear >= MIN_CLEAR_MS
 *                        FEEDER_STOP_COUNT -> feeder stops, MAX_POLLEN -> all stop + flap CLOSED
 *                        not counted while shooting, reversing or at full speed
 *   MODE (limit 1 + 2) - nectar arrives (both pressed) -> NECTAR
 *                        (shooting: at once, otherwise after NECTAR_DETECT_MS + flap CLOSED)
 *                        nectar leaves (both released NECTAR_CLEAR_S) -> POLLEN
 *   SHOT FEED          - every nectar on both switches holds the feed NECTAR_INTERVAL_S,
 *                        and the feed is cut here the moment it is blocked
 */
public class FinalSensorThread2 {

    private final FinalHardware robot;
    private final FinalIntake2Subsystem intake;
    private final FinalShooter2Subsystem shooter;
    private final Gamepad gamepad;

    private Thread thread;
    private volatile boolean running = false;

    // readouts for telemetry
    public volatile boolean b1Broken = false, b2Broken = false;
    public volatile boolean limit1Pressed = false, limit2Pressed = false;
    public volatile boolean nectarAtSwitch = false;   // debounced: a nectar is on both limit switches
    public volatile double hz = 0;

    public FinalSensorThread2(FinalHardware robot, FinalIntake2Subsystem intake,
                              FinalShooter2Subsystem shooter, Gamepad gamepad) {
        this.robot = robot;
        this.intake = intake;
        this.shooter = shooter;
        this.gamepad = gamepad;
    }

    public void start() {
        if (running) return;
        running = true;
        thread = new Thread(this::watch, "FinalSensorThread2");
        thread.start();
    }

    public void stop() {
        running = false;
        if (thread == null) return;
        thread.interrupt();
        try {
            thread.join(500);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    private void watch() {
        long bothSinceNs = System.nanoTime();   // last time NOT both pressed
        long noneSinceNs = System.nanoTime();   // last time something was pressed
        boolean bothPrev = false;

        long windowStart = System.nanoTime();
        int polls = 0;

        while (running && !Thread.currentThread().isInterrupted()) {
            long now = System.nanoTime();
            boolean shooting = intake.isShooting();
            boolean reversing = intake.reversing;

            //TODO pollen count (b1)
            PollenCounter pollen = intake.pollen;
            boolean broken = robot.isB1Broken();
            int before = pollen.getCount();
            boolean wasFull = pollen.isFull();
            pollen.update(now, broken, !reversing && !shooting
                    && intake.intakeState != FinalIntake2Subsystem.INTAKE.FULL_SPEED);
            if (pollen.getCount() > before) gamepad.rumble(100);
            if (pollen.isFull() && !wasFull) gamepad.rumbleBlips(3);
            b1Broken = broken;
            b2Broken = robot.isB2Broken();

            //TODO limit switches - mode follows the piece on them
            boolean l1 = robot.isLimit1Pressed();
            boolean l2 = robot.isLimit2Pressed();
            limit1Pressed = l1;
            limit2Pressed = l2;
            boolean both = l1 && l2;
            boolean none = !l1 && !l2;
            if (!both) bothSinceNs = now;
            if (!none) noneSinceNs = now;
            double bothMs = (now - bothSinceNs) / 1e6;
            double noneMs = (now - noneSinceNs) / 1e6;

            // shooting: every nectar that presses both switches waits NECTAR_INTERVAL_S (one at a time)
            if (shooting && both && !bothPrev) shooter.holdFeed(NECTAR_INTERVAL_S);
            bothPrev = both;

            if (both && !nectarAtSwitch && (shooting || bothMs >= NECTAR_DETECT_MS)) {
                nectarAtSwitch = true;                       // nectar arrived -> NECTAR
                shooter.setMode(MODE.NECTAR, true);
                if (!shooting) shooter.updateState(FLAP.CLOSE);   // don't cut off an LB shot
            } else if (none && nectarAtSwitch && noneMs >= NECTAR_CLEAR_S * 1000) {
                nectarAtSwitch = false;                      // nectar was shot -> POLLEN
                shooter.setMode(MODE.POLLEN, true);
            }
            // a mode change is coming -> shot feed pauses (adjustReady)
            shooter.switchChanging = (both && !nectarAtSwitch) || (none && nectarAtSwitch);

            // feed must not run right now -> stop it here, don't wait for the main loop
            if (shooting && !reversing && intake.feedBlocked()) {
                robot.intake.setPower(0);
                robot.feeder.setPower(0);
            }

            //TODO delayed NECTAR -> POLLEN adjust servo move
            shooter.applyPendingPollenAdjust(now);

            polls++;
            if (now - windowStart >= 1_000_000_000L) {
                hz = polls * 1e9 / (now - windowStart);
                polls = 0;
                windowStart = now;
            }

            if (WATCHER_SLEEP_MS > 0) {
                try {
                    Thread.sleep(WATCHER_SLEEP_MS);
                } catch (InterruptedException e) {
                    return;
                }
            }
        }
    }
}
