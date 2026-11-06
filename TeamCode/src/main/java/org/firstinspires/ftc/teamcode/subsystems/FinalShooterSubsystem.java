package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.hdware.final_Globals.*;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hdware.FinalHardware;
import org.firstinspires.ftc.teamcode.hdware.VelocityPIDF;

/*
 * Shooter (shooter1 + shooter2), POLLEN / NECTAR mode (hood + adjust servo), flap and servo2.
 *
 * periodic() holds the flywheel at TELE_POLLEN_VELOCITY / TELE_NECTAR_VELOCITY with either:
 *   hub PIDF (new FinalShooterSubsystem(robot)) - the hub's own velocity loop (setVelocity),
 *     gains SHOOTER1/2_HUB_kP + SHOOTER_HUB_kI/kD + SHOOTER1/2_HUB_kF (tuned in FINAL: Shooter Tuning)
 *   our PIDF (new FinalShooterSubsystem(robot, true)) - VelocityPIDF per motor + setPower, same as
 *     FinalShooterPIDFSubsystem / FINAL: Shooter Velocity PIDF Test: shooter1 POLLEN_k* / NECTAR_k*,
 *     shooter2 POLLEN2_kP/kF / NECTAR2_kP/kF (kI / kD shared), velocities with SHOOTER*_ENCODER_REVERSED
 * Directions come from FinalHardware (SHOOTER*_REVERSED), so both motors get the same target.
 * To NECTAR the adjust servo + hood move right away; back to POLLEN they move
 * POLLEN_ADJUST_DELAY_S later unless adjustNow.
 *
 * Mode, flap and the feed hold are also changed by FinalSensorThread, so those methods
 * are synchronized.
 */
public class FinalShooterSubsystem extends SubsystemBase {

    private final FinalHardware robot;
    private double appliedP1 = Double.NaN, appliedP2, appliedI, appliedD, appliedF1, appliedF2;

    // our own PIDF (ownPidf = true): one loop per motor on its own encoder
    private final boolean ownPidf;
    private final VelocityPIDF pidf1 = new VelocityPIDF();
    private final VelocityPIDF pidf2 = new VelocityPIDF();
    private MODE lastPidfMode = MODE.POLLEN;

    /** Hub velocity PIDF. */
    public FinalShooterSubsystem(FinalHardware robot) {
        this(robot, false);
    }

    /** ownPidf = true: our VelocityPIDF + setPower (gains POLLEN_k* / POLLEN2_k*, NECTAR_k* / NECTAR2_k*). */
    public FinalShooterSubsystem(FinalHardware robot, boolean ownPidf) {
        this.robot = robot;
        this.ownPidf = ownPidf;
        if (ownPidf) {
            // our PIDF sends power - the hub must not run its velocity loop
            for (DcMotorEx m : new DcMotorEx[]{robot.shooter1, robot.shooter2}) {
                m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                m.setPower(0);
            }
            return;
        }
        // FinalHardware sets RUN_WITHOUT_ENCODER - the hub velocity loop needs RUN_USING_ENCODER
        for (DcMotorEx m : new DcMotorEx[]{robot.shooter1, robot.shooter2}) {
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m.setVelocity(0);
        }
        applyCoefficients();
    }

    public boolean usesOwnPidf() {
        return ownPidf;
    }

    public VelocityPIDF pidf1() {
        return pidf1;
    }

    public VelocityPIDF pidf2() {
        return pidf2;
    }

    /** Sends each motor's own kP / kF + the shared SHOOTER_HUB_kI/kD, only when they change (live dashboard tuning). */
    private void applyCoefficients() {
        if (SHOOTER1_HUB_kP == appliedP1 && SHOOTER2_HUB_kP == appliedP2 && SHOOTER_HUB_kI == appliedI && SHOOTER_HUB_kD == appliedD
                && SHOOTER1_HUB_kF == appliedF1 && SHOOTER2_HUB_kF == appliedF2) return;
        robot.shooter1.setVelocityPIDFCoefficients(SHOOTER1_HUB_kP, SHOOTER_HUB_kI, SHOOTER_HUB_kD, SHOOTER1_HUB_kF);
        robot.shooter2.setVelocityPIDFCoefficients(SHOOTER2_HUB_kP, SHOOTER_HUB_kI, SHOOTER_HUB_kD, SHOOTER2_HUB_kF);
        appliedP1 = SHOOTER1_HUB_kP;
        appliedP2 = SHOOTER2_HUB_kP;
        appliedI = SHOOTER_HUB_kI;
        appliedD = SHOOTER_HUB_kD;
        appliedF1 = SHOOTER1_HUB_kF;
        appliedF2 = SHOOTER2_HUB_kF;
    }

    private void setShooterVelocity(double ticksPerSecond) {
        robot.shooter1.setVelocity(ticksPerSecond);
        robot.shooter2.setVelocity(ticksPerSecond);
    }

    // our PIDF: each motor on its own encoder (encoder flags applied), voltage compensated, never backwards
    private void runOwnPidf() {
        MODE currentMode = mode;
        if (currentMode != lastPidfMode) { // new target / gains
            pidf1.reset();
            pidf2.reset();
        }
        lastPidfMode = currentMode;

        double v1 = robot.getShooter1Velocity();
        double v2 = robot.getShooter2Velocity();
        if (isOn()) {
            boolean nectar = currentMode == MODE.NECTAR;
            double out1 = nectar
                    ? pidf1.update(target, v1, NECTAR_kP, NECTAR_kI, NECTAR_kD, NECTAR_kF, SHOOTER_I_MAX)
                    : pidf1.update(target, v1, POLLEN_kP, POLLEN_kI, POLLEN_kD, POLLEN_kF, SHOOTER_I_MAX);
            double out2 = nectar
                    ? pidf2.update(target, v2, NECTAR2_kP, NECTAR_kI, NECTAR_kD, NECTAR2_kF, SHOOTER_I_MAX)
                    : pidf2.update(target, v2, POLLEN2_kP, POLLEN_kI, POLLEN_kD, POLLEN2_kF, SHOOTER_I_MAX);
            double comp = SHOOTER_VOLTAGE_COMP && voltage > 1 ? NOMINAL_VOLTAGE / voltage : 1;
            power = Range.clip(out1 * comp, 0, 1);
            power2 = Range.clip(out2 * comp, 0, 1);
        } else {
            power = 0;
            power2 = 0;
        }
        robot.shooter1.setPower(power);
        robot.shooter2.setPower(power2);
    }

    public enum SHOOTER {
        ON,
        OFF
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

    public volatile SHOOTER shooterState = SHOOTER.OFF;
    public volatile MODE mode = MODE.POLLEN;
    public volatile FLAP flapState = null;   // null = not sent yet
    public SERVO2 servo2State = null;

    // shooter readouts for telemetry
    public double target = 0, velocity = 0, power = 0, power2 = 0, voltage = 12; // power2: our PIDF only
    private final ElapsedTime voltageTimer = new ElapsedTime();
    private static final double VOLTAGE_READ_MS = 100; // battery voltage is a single hub read - not every loop

    // shot feed timing, shared with the sensor thread
    private volatile long pollenAdjustAtNs = 0;  // pending NECTAR -> POLLEN adjust / hood move, 0 = none
    private volatile long feedHoldUntilNs = 0;   // shooting: no feed before this (adjust servo moving / nectar interval)
    public volatile boolean switchChanging = false; // set by the sensor thread: a mode change is coming

    //TODO Shooter motor
    public void updateState(SHOOTER currentState) {
        this.shooterState = currentState;
        pidf1.reset();
        pidf2.reset();
    }

    public void toggleShooter() {
        updateState(shooterState == SHOOTER.ON ? SHOOTER.OFF : SHOOTER.ON);
    }

    public boolean isOn() {
        return shooterState == SHOOTER.ON;
    }

    public boolean atSpeed() {
        return isOn() && Math.abs(target - velocity) <= SHOOTER_READY_TOL;
    }

    @Override
    public void periodic() {
        applyPendingPollenAdjust(System.nanoTime());

        target = mode == MODE.NECTAR ? TELE_NECTAR_VELOCITY : TELE_POLLEN_VELOCITY;
        velocity = robot.getShooterVelocity();
        if (voltageTimer.milliseconds() >= VOLTAGE_READ_MS) {
            voltageTimer.reset();
            voltage = robot.voltageSensor.getVoltage(); // hub PIDF: telemetry only, our PIDF: voltage compensation
        }

        if (ownPidf) {
            runOwnPidf();
            return;
        }
        applyCoefficients();
        setShooterVelocity(isOn() ? target : 0);
        power = robot.shooter1.getPower();
    }

    //TODO POLLEN / NECTAR mode (hood + adjust servo)
    public synchronized void setMode(MODE newMode, boolean adjustNow) {
        if (mode == newMode) return;
        mode = newMode;
        if (newMode == MODE.NECTAR || adjustNow) {
            pollenAdjustAtNs = 0; // cancel a pending move back to pollen
            moveAdjust(newMode);
        } else {
            pollenAdjustAtNs = System.nanoTime() + (long) (POLLEN_ADJUST_DELAY_S * 1e9);
        }
    }

    public void toggleMode() {
        setMode(mode == MODE.POLLEN ? MODE.NECTAR : MODE.POLLEN, false);
    }

    /** Puts the hood + adjust servo at the mode's position right now (init). */
    public synchronized void forceMode(MODE newMode) {
        mode = newMode;
        pollenAdjustAtNs = 0;
        moveAdjust(newMode);
    }

    // adjust servo + hood to the mode's position, caller holds the lock
    private void moveAdjust(MODE m) {
        boolean nectar = m == MODE.NECTAR;
        robot.setAdjust(nectar ? ADJUST_NECTAR_POS : ADJUST_POLLEN_POS);
        robot.setHood(nectar ? HOOD_NECTAR_POS : HOOD_POLLEN_POS);
        holdFeed(ADJUST_MOVE_S);
    }

    public synchronized void applyPendingPollenAdjust(long now) {
        if (pollenAdjustAtNs == 0 || now < pollenAdjustAtNs) return;
        pollenAdjustAtNs = 0;
        moveAdjust(MODE.POLLEN);
    }

    /** Shot feed waits at least this long from now (keeps a later hold if there is one). */
    public synchronized void holdFeed(double seconds) {
        feedHoldUntilNs = Math.max(feedHoldUntilNs, System.nanoTime() + (long) (seconds * 1e9));
    }

    /** Shooting may feed: no mode change coming, no hold (adjust servo moving / nectar interval). */
    public boolean adjustReady() {
        return !switchChanging && pollenAdjustAtNs == 0 && System.nanoTime() >= feedHoldUntilNs;
    }

    //TODO Flap servo - only writes when the flap actually changes
    public synchronized void updateState(FLAP currentState) {
        if (flapState == currentState) return;
        this.flapState = currentState;
        robot.flap.setPosition(currentState == FLAP.OPEN ? flapOpen : flapClose);
    }

    //TODO Servo2
    public void updateState(SERVO2 currentState) {
        this.servo2State = currentState;
        robot.servo2.setPosition(currentState == SERVO2.OPEN ? servo2Open : servo2Close);
    }

    public void stop() {
        shooterState = SHOOTER.OFF;
        power = 0;
        power2 = 0;
        if (ownPidf) robot.setShooterPower(0);
        else setShooterVelocity(0);
    }

    public Command shooterCommand(SHOOTER state) {
        return new InstantCommand(() -> updateState(state));
    }

    public Command flapCommand(FLAP state) {
        return new InstantCommand(() -> updateState(state));
    }

    public Command servo2Command(SERVO2 state) {
        return new InstantCommand(() -> updateState(state));
    }

    public Command modeCommand(MODE newMode, boolean adjustNow) {
        return new InstantCommand(() -> setMode(newMode, adjustNow));
    }
}
