package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.hdware.final_Globals.*;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.hdware.FinalHardware;

/*
 * Intake (E motor 0) + feeder (E motor 1), pollen count limits and the shot feed.
 * Intake pulls in with NEGATIVE power, same as sankalp27.
 *
 * periodic() picks the powers, highest priority first:
 *   reversing (X held)  - intake +REVERSE_POWER, feeder -REVERSE_POWER
 *   shooting  (LB)      - feeder -SHOOT_REVERSE_POWER for SHOOT_REVERSE_S, then
 *                         intake -SHOOT_INTAKE_POWER + feeder SHOOT_FEEDER_POWER whenever
 *                         FinalShooterSubsystem.adjustReady(), not before SHOOT_FEED_DELAY_S
 *                         (velocityGate: only while target - velocity <= SHOOT_VELOCITY_TOL,
 *                         otherwise SHOOT_SLOW_FEEDER_POWER / SHOOT_SLOW_INTAKE_POWER)
 *   FULL_SPEED          - intake -FULL_SPEED_POWER, feeder FULL_SPEED_POWER, count limits ignored
 *   ON                  - intake -INTAKE_POWER, feeder FEEDER_POWER (feeder off once feederFull,
 *                         both off once full)
 *   OFF                 - 0
 *
 * pollen (PollenCounter) is updated by FinalSensorThread.
 */
public class FinalIntakeSubsystem extends SubsystemBase {

    private final FinalHardware robot;
    private final FinalShooterSubsystem shooter;

    public FinalIntakeSubsystem(FinalHardware robot, FinalShooterSubsystem shooter) {
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
                if (!shooting && !reversing) robot.feeder.setPower(0);
            }

            @Override
            public void onFull() { // stop both + close the flap right now, don't wait for the main loop
                robot.intake.setPower(0);
                robot.feeder.setPower(0);
                shooter.updateState(FinalShooterSubsystem.FLAP.CLOSE);
            }
        });
    }

    public enum INTAKE {
        ON,
        FULL_SPEED,
        OFF
    }

    public volatile INTAKE intakeState = INTAKE.OFF;
    public volatile boolean reversing = false;   // X held

    // shooting (LB)
    public volatile boolean shooting = false;
    // Final Velocity Gate TeleOp: feed only when the shooter is within SHOOT_VELOCITY_TOL, else slow feed.
    // false (FinalCommandTeleOp) = feed whenever shootFeeding()
    public boolean velocityGate = false;
    // FinalCommandTeleOp: shot feed at INTAKE_POWER / FEEDER_POWER (same as RT in FINAL: Shooter Velocity
    // PIDF Test) instead of SHOOT_INTAKE_POWER / SHOOT_FEEDER_POWER
    public boolean shootAtIntakePowers = false;
    private volatile long shootStartNs = 0;

    // pollen count (b1), updated by the sensor thread
    public final PollenCounter pollen;

    // last commanded powers, for telemetry
    public double intakePower = 0, feederPower = 0;

    //TODO Intake + feeder
    public void updateState(INTAKE currentState) {
        this.intakeState = currentState;
    }

    public boolean isRunning() {
        return intakeState != INTAKE.OFF;
    }

    /** Pollen count back to 0, unlocks a full robot. */
    public void resetCount() {
        pollen.reset();
    }

    /** Both limit switches pressed = a nectar is inside. */
    public boolean nectarInside() {
        return robot.isLimit1Pressed() && robot.isLimit2Pressed();
    }

    //TODO Shooting feed
    public void startShooting() {
        intakeState = INTAKE.OFF;
        intakePower = 0;
        feederPower = 0;
        robot.intake.setPower(0);
        robot.feeder.setPower(0);
        shootStartNs = System.nanoTime();
        shooting = true;
    }

    public void stopShooting() {
        shooting = false;
        intakeState = INTAKE.OFF;
    }

    /** Start of the shot: feeder reversed for SHOOT_REVERSE_S. */
    public boolean shootReversing(long now) {
        return shooting && now - shootStartNs < (long) (SHOOT_REVERSE_S * 1e9);
    }

    /** Shooting, reverse done, past SHOOT_FEED_DELAY_S, and nothing holding the feed. */
    public boolean shootFeeding(long now) {
        long since = now - shootStartNs;
        return shooting && since >= (long) (SHOOT_REVERSE_S * 1e9)
                && since >= (long) (SHOOT_FEED_DELAY_S * 1e9) && shooter.adjustReady();
    }

    /** Velocity drop (target - velocity) small enough to shoot. Always true without velocityGate. */
    public boolean shootSpeedReady() {
        return !velocityGate || shooter.target - shooter.velocity <= SHOOT_VELOCITY_TOL;
    }

    /** Seconds since LB started this shot (0 when not shooting). */
    public double shootElapsedS(long now) {
        return shooting ? (now - shootStartNs) / 1e9 : 0;
    }

    /** Flap open, waiting for SHOOT_FEED_DELAY_S. */
    public boolean shootWaiting(long now) {
        return shooting && now - shootStartNs < (long) (SHOOT_FEED_DELAY_S * 1e9);
    }

    @Override
    public void periodic() {
        INTAKE state = intakeState;
        boolean full = pollen.isFull();
        boolean intakeOn = state == INTAKE.ON && !full;
        boolean feederOn = state == INTAKE.ON && !full && !pollen.isFeederFull();

        if (reversing) {
            intakePower = REVERSE_POWER;
            feederPower = -REVERSE_POWER;
        } else if (shooting) {
            long now = System.nanoTime();
            if (shootReversing(now)) {      // start of the shot: feeder back a little first
                intakePower = 0;
                feederPower = -SHOOT_REVERSE_POWER;
            } else {
                boolean feed = shootFeeding(now);
                if (feed && !shootSpeedReady()) { // velocity gate: shooter still recovering -> slow feed
                    intakePower = -SHOOT_SLOW_INTAKE_POWER;
                    feederPower = SHOOT_SLOW_FEEDER_POWER;
                } else {
                    double shootIntake = shootAtIntakePowers ? INTAKE_POWER : SHOOT_INTAKE_POWER;
                    double shootFeeder = shootAtIntakePowers ? FEEDER_POWER : SHOOT_FEEDER_POWER;
                    intakePower = feed ? -shootIntake : 0;
                    feederPower = feed ? shootFeeder : 0;
                }
            }
        } else if (state == INTAKE.FULL_SPEED) {
            intakePower = -FULL_SPEED_POWER;
            feederPower = FULL_SPEED_POWER;
        } else {
            intakePower = intakeOn ? -INTAKE_POWER : 0;
            feederPower = feederOn ? FEEDER_POWER : 0;
        }
        robot.intake.setPower(intakePower);
        robot.feeder.setPower(feederPower);
    }

    public void stop() {
        intakeState = INTAKE.OFF;
        shooting = false;
        reversing = false;
        robot.intake.setPower(0);
        robot.feeder.setPower(0);
    }

    public Command intakeCommand(INTAKE state) {
        return new InstantCommand(() -> updateState(state));
    }
}
