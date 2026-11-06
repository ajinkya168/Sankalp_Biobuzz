package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;
import static org.firstinspires.ftc.teamcode.hdware.final_Globals.*;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hdware.FinalHardware;
import org.firstinspires.ftc.teamcode.hdware.VelocityPIDF;

/*
 * ============================== FINAL SHOOTER 2 (Ivy + states) ==============================
 *
 * Same flywheel logic as FinalShooterPIDFSubsystem (our VelocityPIDF per motor, voltage
 * compensated, setPower), written WORLDS2026 style: one enum per part, one updateState()
 * per enum, one Ivy command per enum. Also holds the parts FinalCommandTeleOp2 needs
 * from FinalShooterSubsystem: POLLEN / NECTAR mode (hood + adjust servo), flap, servo2.
 *
 *   STATE     VALUES            WHAT IT DOES
 *   SHOOTER   OFF / ON          flywheel held at the MODE's velocity by our PIDF
 *   MODE      POLLEN / NECTAR   velocity, PIDF gains, hood + adjust servo
 *   FLAP      OPEN / CLOSE      flapOpen / flapClose
 *   SERVO2    OPEN / CLOSE      servo2Open / servo2Close
 *
 *   updateState(X)   change a state right now (safe from the sensor thread)
 *   xCommand(X)      the same as an Ivy command, for sequences and buttons
 *   periodic()       call ONCE per loop: reads velocity + battery, runs the PIDF
 *
 * Every updateState() changes only its own part, so commands can be grouped with
 * Ivy parallel() in any order.
 */
public class FinalShooter2Subsystem {

    private final FinalHardware robot;
    private final VelocityPIDF pidf1 = new VelocityPIDF();   // shooter1, own encoder
    private final VelocityPIDF pidf2 = new VelocityPIDF();   // shooter2, own encoder

    public FinalShooter2Subsystem(FinalHardware robot) {
        this.robot = robot;
        // our own PIDF sends power - the hub must not run its velocity loop
        for (DcMotorEx m : new DcMotorEx[]{robot.shooter1, robot.shooter2}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        robot.setShooterPower(0);
        voltage = robot.voltageSensor.getVoltage();
    }

    // =====================================================================================
    //                                      STATES
    // =====================================================================================

    public enum SHOOTER {
        OFF,
        ON
    }

    public enum MODE {
        POLLEN,
        NECTAR
    }

    public enum FLAP {
        OPEN,
        CLOSE
    }

    public enum SERVO2 {
        OPEN,
        CLOSE
    }

    // the sensor thread changes mode + flap, so these are volatile
    public volatile SHOOTER shooterState = SHOOTER.OFF;
    public volatile MODE modeState = MODE.POLLEN;
    public volatile FLAP flapState = null;     // null = not sent yet
    public SERVO2 servo2State = null;

    // =====================================================================================
    //                                 TELEMETRY READOUTS
    // =====================================================================================

    // velocity = average of both motors, used for "at speed"
    public double target = 0, velocity = 0, v1 = 0, v2 = 0, power1 = 0, power2 = 0, voltage = 12;

    private final ElapsedTime voltageTimer = new ElapsedTime();
    private static final double VOLTAGE_READ_MS = 100;   // battery voltage is a hub read - not every loop
    private MODE lastPidfMode = MODE.POLLEN;             // PIDF resets when the mode changes

    // =====================================================================================
    //                                  SHOOT FEED TIMING
    // =====================================================================================
    // shared with the intake + sensor thread: the shot feed waits while the adjust servo moves

    private volatile long pollenAdjustAtNs = 0;   // pending NECTAR -> POLLEN adjust / hood move, 0 = none
    private volatile long feedHoldUntilNs = 0;    // no shot feed before this time
    public volatile boolean switchChanging = false; // set by the sensor thread: a mode change is coming

    // =====================================================================================
    //                                   UPDATE STATE
    // =====================================================================================

    //TODO Shooter motors
    public synchronized void updateState(SHOOTER currentState) {
        if (shooterState != currentState) resetPidf();   // fresh PIDF on every on / off
        this.shooterState = currentState;
        switch (currentState) {
            case ON:
                break;                                    // periodic() runs the PIDF
            case OFF:
                power1 = 0;
                power2 = 0;
                robot.setShooterPower(0);
                break;
        }
    }

    public boolean isOn() {
        return shooterState == SHOOTER.ON;
    }

    public boolean atSpeed() {
        return isOn() && Math.abs(target - velocity) <= SHOOTER_READY_TOL;
    }

    //TODO Mode (velocity + gains + hood + adjust servo)
    /**
     * Changes POLLEN / NECTAR. To NECTAR (or adjustNow) the hood + adjust servo move right away;
     * back to POLLEN they move POLLEN_ADJUST_DELAY_S later (lets the last nectar leave first).
     */
    public synchronized void setMode(MODE newMode, boolean adjustNow) {
        if (modeState == newMode) return;
        modeState = newMode;
        if (newMode == MODE.NECTAR || adjustNow) {
            pollenAdjustAtNs = 0;                         // cancel a pending move back to pollen
            moveAdjust(newMode);
        } else {
            pollenAdjustAtNs = System.nanoTime() + (long) (POLLEN_ADJUST_DELAY_S * 1e9);
        }
    }

    /** Puts the hood + adjust servo at the mode's position right now, even if the mode didn't change (init). */
    public synchronized void forceMode(MODE newMode) {
        modeState = newMode;
        pollenAdjustAtNs = 0;
        moveAdjust(newMode);
    }

    // adjust servo + hood to the mode's position, then hold the shot feed while they move
    private void moveAdjust(MODE m) {
        switch (m) {
            case POLLEN:
                robot.setAdjust(ADJUST_POLLEN_POS);
                robot.setHood(HOOD_POLLEN_POS);
                break;
            case NECTAR:
                robot.setAdjust(ADJUST_NECTAR_POS);
                robot.setHood(HOOD_NECTAR_POS);
                break;
        }
        holdFeed(ADJUST_MOVE_S);
    }

    /** Runs the delayed NECTAR -> POLLEN servo move once its time comes. */
    public synchronized void applyPendingPollenAdjust(long now) {
        if (pollenAdjustAtNs == 0 || now < pollenAdjustAtNs) return;
        pollenAdjustAtNs = 0;
        moveAdjust(MODE.POLLEN);
    }

    //TODO Flap - only writes when it actually changes (each servo write is a hub round trip)
    public synchronized void updateState(FLAP currentState) {
        if (flapState == currentState) return;
        this.flapState = currentState;
        double pos = flapClose;
        switch (currentState) {
            case OPEN:
                pos = flapOpen;
                break;
            case CLOSE:
                pos = flapClose;
                break;
        }
        robot.flap.setPosition(pos);
    }

    //TODO Servo2
    public void updateState(SERVO2 currentState) {
        this.servo2State = currentState;
        double pos = servo2Close;
        switch (currentState) {
            case OPEN:
                pos = servo2Open;
                break;
            case CLOSE:
                pos = servo2Close;
                break;
        }
        robot.servo2.setPosition(pos);
    }

    // =====================================================================================
    //                                  FEED HOLD (shot)
    // =====================================================================================

    /** Shot feed waits at least this long from now (keeps a later hold if there is one). */
    public synchronized void holdFeed(double seconds) {
        feedHoldUntilNs = Math.max(feedHoldUntilNs, System.nanoTime() + (long) (seconds * 1e9));
    }

    /** Shot may feed: no mode change coming, no servo move pending, no hold. */
    public boolean adjustReady() {
        return !switchChanging && pollenAdjustAtNs == 0 && System.nanoTime() >= feedHoldUntilNs;
    }

    // =====================================================================================
    //                              PERIODIC (once per loop)
    // =====================================================================================

    public void periodic() {
        applyPendingPollenAdjust(System.nanoTime());

        // read
        v1 = robot.getShooter1Velocity();
        v2 = robot.getShooter2Velocity();
        velocity = (v1 + v2) / 2;
        if (voltageTimer.milliseconds() >= VOLTAGE_READ_MS) {
            voltageTimer.reset();
            voltage = robot.voltageSensor.getVoltage();
        }

        // new mode = new target + gains -> start the PIDF fresh
        MODE mode = modeState;
        if (mode != lastPidfMode) resetPidf();
        lastPidfMode = mode;

        // run
        switch (shooterState) {
            case ON:
                target = modeVelocity(mode);
                runPidf(mode);
                break;
            case OFF:
                target = 0;
                power1 = 0;
                power2 = 0;
                break;
        }
        robot.shooter1.setPower(power1);
        robot.shooter2.setPower(power2);
    }

    private static double modeVelocity(MODE mode) {
        switch (mode) {
            case NECTAR:
                return TELE_NECTAR_VELOCITY;
            case POLLEN:
            default:
                return TELE_POLLEN_VELOCITY;
        }
    }

    // one PIDF per motor on its own encoder, gains from the mode, voltage compensated
    private void runPidf(MODE mode) {
        double out1, out2;
        switch (mode) {
            case NECTAR:
                out1 = pidf1.update(target, v1, NECTAR_kP, NECTAR_kI, NECTAR_kD, NECTAR_kF, SHOOTER_I_MAX);
                out2 = pidf2.update(target, v2, NECTAR2_kP, NECTAR_kI, NECTAR_kD, NECTAR2_kF, SHOOTER_I_MAX);
                break;
            case POLLEN:
            default:
                out1 = pidf1.update(target, v1, POLLEN_kP, POLLEN_kI, POLLEN_kD, POLLEN_kF, SHOOTER_I_MAX);
                out2 = pidf2.update(target, v2, POLLEN2_kP, POLLEN_kI, POLLEN_kD, POLLEN2_kF, SHOOTER_I_MAX);
                break;
        }
        // battery high -> less power, battery low -> more power (see NOMINAL_VOLTAGE)
        double comp = SHOOTER_VOLTAGE_COMP && voltage > 1 ? NOMINAL_VOLTAGE / voltage : 1;
        power1 = Range.clip(out1 * comp, 0, 1);   // never drive the flywheel backwards
        power2 = Range.clip(out2 * comp, 0, 1);
    }

    private void resetPidf() {
        pidf1.reset();
        pidf2.reset();
    }

    public VelocityPIDF pidf1() {
        return pidf1;
    }

    public VelocityPIDF pidf2() {
        return pidf2;
    }

    public void stop() {
        shooterState = SHOOTER.OFF;
        resetPidf();
        power1 = 0;
        power2 = 0;
        robot.setShooterPower(0);
    }

    // =====================================================================================
    //                                  COMMANDS (Ivy)
    // =====================================================================================

    public Command shooterCommand(SHOOTER state) {
        return instant(() -> updateState(state));
    }

    public Command toggleShooterCommand() {
        return instant(() -> updateState(isOn() ? SHOOTER.OFF : SHOOTER.ON));
    }

    public Command modeCommand(MODE mode, boolean adjustNow) {
        return instant(() -> setMode(mode, adjustNow));
    }

    public Command forceModeCommand(MODE mode) {
        return instant(() -> forceMode(mode));
    }

    /** BACK button: POLLEN <-> NECTAR (back to POLLEN waits POLLEN_ADJUST_DELAY_S). */
    public Command toggleModeCommand() {
        return instant(() -> setMode(modeState == MODE.POLLEN ? MODE.NECTAR : MODE.POLLEN, false));
    }

    public Command flapCommand(FLAP state) {
        return instant(() -> updateState(state));
    }

    public Command servo2Command(SERVO2 state) {
        return instant(() -> updateState(state));
    }
}
