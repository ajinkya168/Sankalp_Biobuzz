package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;
import static org.firstinspires.ftc.teamcode.hdware.final_Globals.*;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.hdware.FinalHardware;
import org.firstinspires.ftc.teamcode.subsystems.FinalShooter2Subsystem.FLAP;

/*
 * ============================== FINAL INTAKE 2 (Ivy + states) ==============================
 *
 * Intake (E motor 0) + feeder (E motor 1), the pollen count and the LB shot, as states.
 * Same behaviour as FinalIntakeSubsystem in FinalCommandTeleOp. Intake pulls in with NEGATIVE power.
 *
 *   STATE    VALUES                  WHAT IT DOES
 *   INTAKE   OFF / ON / FULL_SPEED   normal intake (ON obeys the pollen count limits)
 *   SHOT     IDLE                    not shooting
 *            KICKBACK                feeder back SHOOT_REVERSE_POWER for SHOOT_REVERSE_S
 *            WAIT                    flap open, nothing moving until SHOOT_FEED_DELAY_S
 *            FEEDING                 intake + feeder push pieces into the shooter
 *            PAUSED                  feed stopped while the adjust servo moves / nectar interval
 *   + reversing (X held)             beats everything: intake + feeder backwards
 *
 * SHOT state machine (runs in periodic):
 *
 *   LB -> KICKBACK --SHOOT_REVERSE_S--> WAIT --SHOOT_FEED_DELAY_S--> FEEDING <--> PAUSED
 *                                                                     (adjustReady)
 *   any state --SHOOT_FLAP_CLOSE_S after LB--> IDLE + flap CLOSE  (auto close, 0 = off)
 *   LB again / B / A                        --> IDLE
 *
 * The pollen count (PollenCounter) is updated by FinalSensorThread2.
 */
public class FinalIntake2Subsystem {

    private final FinalHardware robot;
    private final FinalShooter2Subsystem shooter;

    public FinalIntake2Subsystem(FinalHardware robot, FinalShooter2Subsystem shooter) {
        this.robot = robot;
        this.shooter = shooter;
        pollen = new PollenCounter(new PollenCounter.Params() {
            public double minBreakMs() { return MIN_BREAK_MS; }
            public double minClearMs() { return MIN_CLEAR_MS; }
            public int feederStopCount() { return FEEDER_STOP_COUNT; }
            public double feederStopDelayMs() { return FEEDER_STOP_DELAY_MS; }
            public int maxCount() { return MAX_POLLEN; }
        }, new PollenCounter.Listener() {
            @Override
            public void onFeederStop() {
                if (!isShooting() && !reversing) robot.feeder.setPower(0);
            }

            @Override
            public void onFull() { // stop both + close the flap right now, don't wait for the main loop
                robot.intake.setPower(0);
                robot.feeder.setPower(0);
                shooter.updateState(FLAP.CLOSE);
            }
        });
    }

    // =====================================================================================
    //                                      STATES
    // =====================================================================================

    public enum INTAKE {
        OFF,
        ON,
        FULL_SPEED
    }

    public enum SHOT {
        IDLE,
        KICKBACK,
        WAIT,
        FEEDING,
        PAUSED
    }

    // the sensor thread reads these, so they are volatile
    public volatile INTAKE intakeState = INTAKE.OFF;
    public volatile SHOT shotState = SHOT.IDLE;
    public volatile boolean reversing = false;   // X held

    // pollen count (beam b1), updated by the sensor thread
    public final PollenCounter pollen;

    // readouts for telemetry
    public double intakePower = 0, feederPower = 0;
    private volatile long shotStartNs = 0;

    // =====================================================================================
    //                                   UPDATE STATE
    // =====================================================================================

    //TODO Intake + feeder
    public void updateState(INTAKE currentState) {
        this.intakeState = currentState;
    }

    public boolean isRunning() {
        return intakeState != INTAKE.OFF;
    }

    //TODO Shot (LB)
    /** KICKBACK = start a shot (intake off, timer starts). IDLE = stop the shot. */
    public void updateState(SHOT currentState) {
        switch (currentState) {
            case KICKBACK:                       // start of a shot
                intakeState = INTAKE.OFF;
                setPowers(0, 0);
                shotStartNs = System.nanoTime();
                break;
            case IDLE:
            default:
                break;
        }
        this.shotState = currentState;
    }

    public boolean isShooting() {
        return shotState != SHOT.IDLE;
    }

    /** Seconds since LB started this shot (0 when not shooting). */
    public double shotSeconds(long now) {
        return isShooting() ? (now - shotStartNs) / 1e9 : 0;
    }

    /**
     * Shot running but the feed must be stopped right now (WAIT / PAUSED / a hold just started).
     * Safe from the sensor thread, which uses it to cut the motors without waiting for the loop.
     */
    public boolean feedBlocked() {
        SHOT s = shotState;
        return s == SHOT.WAIT || s == SHOT.PAUSED || (s == SHOT.FEEDING && !shooter.adjustReady());
    }

    //TODO Pollen count
    /** Pollen count back to 0, unlocks a full robot. */
    public void resetCount() {
        pollen.reset();
    }

    /** Both limit switches pressed = a nectar is inside. */
    public boolean nectarInside() {
        return robot.isLimit1Pressed() && robot.isLimit2Pressed();
    }

    // =====================================================================================
    //                              PERIODIC (once per loop)
    // =====================================================================================

    public void periodic() {
        updateShot(System.nanoTime());

        // highest priority first: X reverse > shot > normal intake
        if (reversing) {
            setPowers(REVERSE_POWER, -REVERSE_POWER);
        } else if (isShooting()) {
            shotPowers();
        } else {
            intakePowers();
        }
    }

    // SHOT state machine - moves to the next state when its time / condition comes
    private void updateShot(long now) {
        if (!isShooting()) return;
        double t = shotSeconds(now);

        // auto close: the whole shot ends SHOOT_FLAP_CLOSE_S after LB
        if (SHOOT_FLAP_CLOSE_S > 0 && t >= SHOOT_FLAP_CLOSE_S) {
            updateState(SHOT.IDLE);
            shooter.updateState(FLAP.CLOSE);
            return;
        }

        switch (shotState) {
            case KICKBACK:
                if (t >= SHOOT_REVERSE_S) shotState = SHOT.WAIT;
                break;
            case WAIT:
                if (t >= SHOOT_FEED_DELAY_S) shotState = shooter.adjustReady() ? SHOT.FEEDING : SHOT.PAUSED;
                break;
            case FEEDING:
                if (!shooter.adjustReady()) shotState = SHOT.PAUSED;
                break;
            case PAUSED:
                if (shooter.adjustReady()) shotState = SHOT.FEEDING;
                break;
            default:
                break;
        }
    }

    // powers for each SHOT state (shot feed at INTAKE_POWER / FEEDER_POWER, same as FinalCommandTeleOp)
    private void shotPowers() {
        switch (shotState) {
            case KICKBACK:
                setPowers(0, -SHOOT_REVERSE_POWER);
                break;
            case FEEDING:
                setPowers(-INTAKE_POWER, FEEDER_POWER);
                break;
            case WAIT:
            case PAUSED:
            default:
                setPowers(0, 0);
                break;
        }
    }

    // powers for each INTAKE state (ON stops the feeder / everything at the pollen count limits)
    private void intakePowers() {
        boolean full = pollen.isFull();
        switch (intakeState) {
            case ON:
                setPowers(full ? 0 : -INTAKE_POWER,
                        full || pollen.isFeederFull() ? 0 : FEEDER_POWER);
                break;
            case FULL_SPEED:
                setPowers(-FULL_SPEED_POWER, FULL_SPEED_POWER);
                break;
            case OFF:
            default:
                setPowers(0, 0);
                break;
        }
    }

    private void setPowers(double intake, double feeder) {
        intakePower = intake;
        feederPower = feeder;
        robot.intake.setPower(intake);
        robot.feeder.setPower(feeder);
    }

    public void stop() {
        intakeState = INTAKE.OFF;
        shotState = SHOT.IDLE;
        reversing = false;
        setPowers(0, 0);
    }

    // =====================================================================================
    //                                  COMMANDS (Ivy)
    // =====================================================================================

    public Command intakeCommand(INTAKE state) {
        return instant(() -> updateState(state));
    }

    public Command shotCommand(SHOT state) {
        return instant(() -> updateState(state));
    }

    public Command resetCountCommand() {
        return instant(this::resetCount);
    }

    /** X: reverses while held, stops reversing the moment it's released - use with Bindings.whileTrue. */
    public Command reverseWhileHeld() {
        return Command.build()
                .setStart(() -> reversing = true)
                .setDone(() -> false)
                .setEnd(endCondition -> reversing = false);
    }
}
