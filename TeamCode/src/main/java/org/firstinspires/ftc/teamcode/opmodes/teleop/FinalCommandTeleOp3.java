package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static org.firstinspires.ftc.teamcode.hdware.final_Globals.*;
import static org.firstinspires.ftc.teamcode.sequences.FinalSequences2.fullSpeed;
import static org.firstinspires.ftc.teamcode.sequences.FinalSequences2.initSequence;
import static org.firstinspires.ftc.teamcode.sequences.FinalSequences2.intakeStop;
import static org.firstinspires.ftc.teamcode.sequences.FinalSequences2.intakeToggle;
import static org.firstinspires.ftc.teamcode.sequences.FinalSequences2.modeToggle;
import static org.firstinspires.ftc.teamcode.sequences.FinalSequences2.shoot;
import static org.firstinspires.ftc.teamcode.sequences.FinalSequences2.shooterToggle;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.utils.Angle;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hdware.FinalHardware;
import org.firstinspires.ftc.teamcode.ivy.Bindings;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.FinalIntake2Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.FinalSensorThread2;
import org.firstinspires.ftc.teamcode.subsystems.FinalShooter2Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.FinalShooter2Subsystem.SERVO2;
import org.firstinspires.ftc.teamcode.subsystems.FinalShooter2Subsystem.SHOOTER;
import org.firstinspires.ftc.teamcode.subsystems.FinalIntake2Subsystem.INTAKE;

/*
 * ============================== FINAL COMMAND TELEOP 3 (Ramp Full) ==============================
 *
 * Copy of FinalCommandTeleOp2 + ONE new rule (everything else is the same):
 *
 *   RAMP FULL: while the intake is running, if BOTH beam breaks (B1 + B2) stay broken for
 *   RAMP_FULL_S (FTC Dashboard, default 1 s) -> all pollen is on the ramp:
 *       intake + feeder OFF, shooter ON (spins up, ready for LB).
 *   Fires once; it re-arms when either beam clears. Not while shooting or reversing (X).
 *
 * FinalCommandTeleOp2 description:
 * Same robot behaviour as FinalCommandTeleOp, written with:
 *   - Ivy commands + Bindings (no FTCLib): every button is ONE line in bindButtons()
 *   - state machines: every part is an enum state in its subsystem
 *       FinalShooter2Subsystem  SHOOTER / MODE / FLAP / SERVO2
 *       FinalIntake2Subsystem   INTAKE / SHOT (the LB shot state machine)
 *       this TeleOp             DRIVE (field / robot centric)
 *   - button actions in FinalSequences2, all numbers in final_Globals (FTC Dashboard)
 *
 * ---------------------------------- GAMEPAD 1 ----------------------------------
 *   DRIVE
 *     Left stick         drive (strafe x StrafeSpeed)
 *     Right stick        turn (x turnSpeed)
 *     Right stick click  reset heading (the way the robot faces now = field "forward")
 *     Right bumper       field centric <-> robot centric
 *
 *   INTAKE
 *     A                  intake + feeder on / off
 *                        (on: pollen count -> 0, flap CLOSED, NECTAR if one is inside, else POLLEN)
 *     X (hold)           reverse intake + feeder
 *     Dpad left          stop intake + feeder
 *     Dpad right         full speed feed, flap OPEN (only when stopped)
 *
 *   SHOOTER
 *     B                  shooter on / off (back to POLLEN, intake + feeder off)
 *     BACK               POLLEN <-> NECTAR
 *     Left bumper        SHOOT: KICKBACK -> WAIT -> FEEDING, flap closes by itself after
 *                        SHOOT_FLAP_CLOSE_S, or press LB again to stop
 *     Dpad up / down     servo2 CLOSE / OPEN
 *
 * ---------------------------------- EVERY LOOP ----------------------------------
 *   bindings.poll()       buttons -> schedule commands (commands change states)
 *   Scheduler.execute()   run / finish the commands
 *   periodic()            each subsystem turns its states into motor powers
 */
@Config
@TeleOp(name = "Final Command TeleOp 3 (Ramp Full)", group = "Final")
public class FinalCommandTeleOp3 extends OpMode {

    // ------------------------------ RAMP FULL (new) - tune on FTC Dashboard ------------------------------
    /** Both beam breaks must stay broken this long (seconds) before intake/feeder stop + shooter starts. */
    public static double RAMP_FULL_S = 0.05;

    private final ElapsedTime rampTimer = new ElapsedTime();
    private boolean rampBothBroken = false;   // both beams broken in the last loop
    private boolean rampFired = false;        // already acted for this "both broken" period

    // ------------------------------ hardware + subsystems ------------------------------
    private final FinalHardware robot = FinalHardware.getInstance();
    private Follower follower;
    private FinalShooter2Subsystem shooterSS;
    private FinalIntake2Subsystem intakeSS;
    private FinalSensorThread2 sensors;

    // ------------------------------ buttons -> commands ------------------------------
    private final Bindings bindings = new Bindings();

    // ------------------------------ drive state ------------------------------
    public enum DRIVE {
        FIELD_CENTRIC,
        ROBOT_CENTRIC
    }

    private DRIVE driveState = DRIVE.FIELD_CENTRIC;
    private double headingOffset = 0.0;     // radians that count as "forward", set by right stick click

    // ------------------------------ turret (held fixed) ------------------------------
    private double turretSentPos = Double.NaN;

    // ------------------------------ loop timing ------------------------------
    private final ElapsedTime loopTimer = new ElapsedTime();

    // ------------------------------ motor current readouts ------------------------------
    // each getCurrent() is its own hub read (slow), so we only read them every CURRENT_READ_MS
    private static final String[] MOTOR_NAMES = {"fl", "fr", "bl", "br", "intake", "feeder", "shooter1", "shooter2"};
    private static final double CURRENT_READ_MS = 100;
    private DcMotorEx[] currentMotors;
    private final double[] motorAmps = new double[MOTOR_NAMES.length];
    private final ElapsedTime currentTimer = new ElapsedTime();

    // =====================================================================================
    //                                       INIT
    // =====================================================================================

    @Override
    public void init() {
        Scheduler.reset();

        // hardware + subsystems
        robot.init(hardwareMap);
        follower = Constants.create(hardwareMap);
        shooterSS = new FinalShooter2Subsystem(robot);
        intakeSS = new FinalIntake2Subsystem(robot, shooterSS);
        sensors = new FinalSensorThread2(robot, intakeSS, shooterSS, gamepad1);

        // bulk reads: one hub read per loop for all encoders / sensors
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        currentMotors = new DcMotorEx[MOTOR_NAMES.length];
        for (int i = 0; i < MOTOR_NAMES.length; i++) {
            currentMotors[i] = hardwareMap.get(DcMotorEx.class, MOTOR_NAMES[i]);
        }

        bindButtons();

        telemetry.addLine("Final Command TeleOp 3 (Ramp Full) ready - POLLEN");
        telemetry.update();
    }

    /** Every button, once. Bindings does the "just pressed" check for us. */
    private void bindButtons() {
        // DRIVE
        bindings.onTrue(() -> gamepad1.right_bumper, instant(this::toggleDrive));
        bindings.onTrue(() -> gamepad1.right_stick_button, instant(() -> headingOffset = follower.pose().heading()));

        // INTAKE
        bindings.onTrue(() -> gamepad1.a, intakeToggle(intakeSS, shooterSS));
        bindings.whileTrue(() -> gamepad1.x, intakeSS.reverseWhileHeld());
        bindings.onTrue(() -> gamepad1.dpad_left, intakeStop(intakeSS, shooterSS));
        bindings.onTrue(() -> gamepad1.dpad_right, fullSpeed(intakeSS, shooterSS));

        // SHOOTER
        bindings.onTrue(() -> gamepad1.b, shooterToggle(intakeSS, shooterSS));
        bindings.onTrue(() -> gamepad1.back, modeToggle(shooterSS));
        bindings.onTrue(() -> gamepad1.left_bumper, shoot(intakeSS, shooterSS, sensors));

        // SERVO2
        bindings.onTrue(() -> gamepad1.dpad_up, shooterSS.servo2Command(SERVO2.CLOSE));
        bindings.onTrue(() -> gamepad1.dpad_down, shooterSS.servo2Command(SERVO2.OPEN));
    }

    // =====================================================================================
    //                                      START
    // =====================================================================================

    @Override
    public void start() {
        Scheduler.schedule(initSequence(intakeSS, shooterSS));
        sensors.start();
        loopTimer.reset();
    }

    // =====================================================================================
    //                                    MAIN LOOP
    // =====================================================================================

    @Override
    public void loop() {
        double loopMs = loopTimer.milliseconds();
        loopTimer.reset();

        bindings.poll();          // buttons -> schedule commands
        checkRampFull();          // NEW: both beams broken long enough -> intake/feeder off, shooter on
        Scheduler.execute();      // run / finish commands

        shooterSS.periodic();     // SHOOTER + MODE -> flywheel power
        intakeSS.periodic();      // INTAKE + SHOT  -> intake / feeder power
        holdTurret();
        drive();

        showTelemetry(loopMs);
    }

    // =====================================================================================
    //                              RAMP FULL (new logic)
    // =====================================================================================

    /**
     * Both beam breaks broken for RAMP_FULL_S while intaking = all pollen on the ramp
     * -> intake + feeder OFF, shooter ON. Once per "both broken" period.
     */
    private void checkRampFull() {
        boolean bothBroken = sensors.b1Broken && sensors.b2Broken;

        if (!bothBroken) {                       // a beam cleared -> reset + re-arm
            rampBothBroken = false;
            rampFired = false;
            return;
        }
        if (!rampBothBroken) {                   // just became both broken -> start timing
            rampBothBroken = true;
            rampTimer.reset();
        }

        boolean intaking = intakeSS.isRunning() && !intakeSS.isShooting() && !intakeSS.reversing;
        if (!rampFired && intaking && rampTimer.seconds() >= RAMP_FULL_S) {
            rampFired = true;
            Scheduler.schedule(parallel(
                    intakeSS.intakeCommand(INTAKE.OFF),       // intake + feeder off
                    shooterSS.shooterCommand(SHOOTER.ON)      // spin up, ready for LB
            ));
        }
    }

    /** Telemetry text for the ramp-full rule. */
    private String rampStatus() {
        if (rampFired) return "FULL - intake off, shooter on";
        if (rampBothBroken) return String.format("both broken %.1f / %.1f s", rampTimer.seconds(), RAMP_FULL_S);
        return "waiting";
    }

    // =====================================================================================
    //                                       STOP
    // =====================================================================================

    @Override
    public void stop() {
        if (sensors != null) sensors.stop();
        if (intakeSS != null) intakeSS.stop();
        if (shooterSS != null) shooterSS.stop();
        if (follower != null) follower.stop();
        Scheduler.reset();
    }

    // =====================================================================================
    //                                      DRIVE
    // =====================================================================================

    private void toggleDrive() {
        driveState = driveState == DRIVE.FIELD_CENTRIC ? DRIVE.ROBOT_CENTRIC : DRIVE.FIELD_CENTRIC;
    }

    private void drive() {
        // our sticks: forward +, strafe right +, turn right +
        double y = deadzone(-gamepad1.left_stick_y);
        double x = deadzone(gamepad1.left_stick_x) * StrafeSpeed;
        double turn = deadzone(gamepad1.right_stick_x) * turnSpeed;

        double heading = 0.0;
        switch (driveState) {
            case FIELD_CENTRIC:
                heading = follower.pose().heading() - headingOffset;
                break;
            case ROBOT_CENTRIC:
                heading = 0.0;
                break;
        }

        // Pedro wants: forward +, strafe LEFT +, turn COUNTER-clockwise + (so x and turn are flipped)
        follower.manual(ManualDrive.fieldCentric(y, -x, -turn, heading));
        follower.update();
    }

    private static double deadzone(double value) {
        return Math.abs(value) < 0.05 ? 0.0 : value;
    }

    // turret held at TELE_TURRET_POS, re-sent only when the dashboard value changes
    private void holdTurret() {
        if (TELE_TURRET_POS == turretSentPos) return;
        robot.setTurretPosition(TELE_TURRET_POS);
        turretSentPos = TELE_TURRET_POS;
    }

    // =====================================================================================
    //                                    TELEMETRY
    // =====================================================================================

    /** Driver Station text. Ends by sending the FTC Dashboard graphs. */
    private void showTelemetry(double loopMs) {
        long now = System.nanoTime();
        readMotorCurrents();

        telemetry.addData("Loop time (ms)", "%.1f  (%.0f Hz)", loopMs, loopMs > 0 ? 1000 / loopMs : 0);

        // shooter
        telemetry.addData("MODE", shooterSS.modeState);
        telemetry.addData("SHOOTER (B)", "%s%s", shooterSS.shooterState,
                shooterSS.isOn() ? (shooterSS.atSpeed() ? " - AT SPEED" : " - spinning up") : "");
        telemetry.addData("  target / velocity / power 1 / power 2", "%.0f / %.0f / %.3f / %.3f",
                shooterSS.target, shooterSS.velocity, shooterSS.power1, shooterSS.power2);
        telemetry.addData("SHOT (LB)", shotStatus(now));

        // intake + pollen count
        telemetry.addData("POLLEN COUNT", "%d / %d%s", intakeSS.pollen.getCount(), MAX_POLLEN, pollenNote());
        telemetry.addData("INTAKE", "%s  (%.2f)", intakeSS.reversing ? "REVERSE (X)" : intakeSS.intakeState, intakeSS.intakePower);
        telemetry.addData("Feeder", "%.2f", intakeSS.feederPower);

        // motor currents
        telemetry.addData("Current (A) drive fl / fr / bl / br", "%.2f / %.2f / %.2f / %.2f",
                motorAmps[0], motorAmps[1], motorAmps[2], motorAmps[3]);
        telemetry.addData("Current (A) intake / feeder", "%.2f / %.2f", motorAmps[4], motorAmps[5]);
        telemetry.addData("Current (A) shooter1 / shooter2", "%.2f / %.2f", motorAmps[6], motorAmps[7]);

        // servos + sensors
        telemetry.addData("FLAP / SERVO2", "%s / %s", shooterSS.flapState, shooterSS.servo2State);
        telemetry.addData("B1 / B2", "%s / %s", sensors.b1Broken ? "BROKEN" : "clear", sensors.b2Broken ? "BROKEN" : "clear");
        telemetry.addData("RAMP FULL", rampStatus());
        telemetry.addData("Limit 1 / 2", "%s / %s  %s", sensors.limit1Pressed ? "PRESSED" : "released",
                sensors.limit2Pressed ? "PRESSED" : "released", sensors.nectarAtSwitch ? "(NECTAR on switches)" : "");
        telemetry.addData("Turret pos", "%.3f", turretSentPos);

        // drive
        telemetry.addData("DRIVE (RB)", driveState);
        telemetry.addData("Heading", "%.1f",
                Math.toDegrees(Angle.normalizeSigned(follower.pose().heading() - headingOffset)));
        telemetry.addData("Pose", follower.pose());
        telemetry.addData("Sensor thread (Hz)", "%.0f", sensors.hz);

        telemetry.addLine("A intake | B shooter | BACK pollen/nectar | LB shoot | X reverse | dpad L stop | dpad R full speed | dpad U/D servo2 | RB field/robot | R-stick zero heading");
        telemetry.update();

        sendGraphs(loopMs, now);
    }

    /** SHOT state, plus time until the flap auto closes. */
    private String shotStatus(long now) {
        if (!intakeSS.isShooting()) return "IDLE";
        if (SHOOT_FLAP_CLOSE_S <= 0) return intakeSS.shotState.toString();
        double left = Math.max(0, SHOOT_FLAP_CLOSE_S - intakeSS.shotSeconds(now));
        return String.format("%s  (flap closes in %.1f s)", intakeSS.shotState, left);
    }

    /** Extra note after the pollen count (full / feeder stopped / stopping). */
    private String pollenNote() {
        if (intakeSS.pollen.isFull()) return "  FULL - LB to shoot / A to reset";
        if (intakeSS.pollen.isFeederFull()) return "  feeder stopped";
        if (intakeSS.pollen.isFeederStopping()) return "  feeder stopping...";
        return "";
    }

    private void readMotorCurrents() {
        if (currentTimer.milliseconds() < CURRENT_READ_MS) return;
        currentTimer.reset();
        for (int i = 0; i < currentMotors.length; i++) {
            motorAmps[i] = currentMotors[i].getCurrent(CurrentUnit.AMPS);
        }
    }

    // =====================================================================================
    //                              FTC DASHBOARD GRAPHS
    // =====================================================================================

    /*
     * Numbers only (states sent as numbers so they graph). Suggested graphs:
     *   velocity : shooter target, shooter velocity
     *   states   : shot state (0 IDLE, 1 KICKBACK, 2 WAIT, 3 FEEDING, 4 PAUSED), flap open, nectar mode
     *   tuning   : the final_Globals values (flat lines - shows when you change one)
     */
    private void sendGraphs(double loopMs, long now) {
        boolean nectar = shooterSS.modeState == FinalShooter2Subsystem.MODE.NECTAR;

        TelemetryPacket packet = new TelemetryPacket();

        // shooter velocity
        packet.put("shooter target", shooterSS.target);
        packet.put("shooter velocity", shooterSS.velocity);
        packet.put("shooter error", shooterSS.target - shooterSS.velocity);
        packet.put("shooter1 velocity", shooterSS.v1);
        packet.put("shooter2 velocity", shooterSS.v2);
        packet.put("shooter1 power", shooterSS.power1);
        packet.put("shooter2 power", shooterSS.power2);
        packet.put("shooter1 P term", shooterSS.pidf1().pTerm);
        packet.put("shooter2 P term", shooterSS.pidf2().pTerm);
        packet.put("battery voltage", shooterSS.voltage);

        // states
        packet.put("shot state", intakeSS.shotState.ordinal());
        packet.put("shot time (s)", intakeSS.shotSeconds(now));
        packet.put("flap open", shooterSS.flapState == FinalShooter2Subsystem.FLAP.OPEN ? 1 : 0);
        packet.put("nectar mode", nectar ? 1 : 0);
        packet.put("nectar at switch", sensors.nectarAtSwitch ? 1 : 0);
        packet.put("limit 1", sensors.limit1Pressed ? 1 : 0);
        packet.put("limit 2", sensors.limit2Pressed ? 1 : 0);

        // intake / feeder / adjust
        packet.put("pollen count", intakeSS.pollen.getCount());
        packet.put("intake power", intakeSS.intakePower);
        packet.put("feeder power", intakeSS.feederPower);
        packet.put("adjust position", robot.adjust.getPosition());

        // motor current (A), refreshed every CURRENT_READ_MS
        for (int i = 0; i < MOTOR_NAMES.length; i++) packet.put("current " + MOTOR_NAMES[i], motorAmps[i]);

        // tuning values (final_Globals)
        packet.put("TELE_POLLEN_VELOCITY", TELE_POLLEN_VELOCITY);
        packet.put("TELE_NECTAR_VELOCITY", TELE_NECTAR_VELOCITY);
        packet.put("SHOOT_FEED_DELAY_S", SHOOT_FEED_DELAY_S);
        packet.put("SHOOT_REVERSE_S", SHOOT_REVERSE_S);
        packet.put("SHOOT_REVERSE_POWER", SHOOT_REVERSE_POWER);
        packet.put("SHOOT_FLAP_CLOSE_S", SHOOT_FLAP_CLOSE_S);
        packet.put("ADJUST_MOVE_S", ADJUST_MOVE_S);
        packet.put("NECTAR_INTERVAL_S", NECTAR_INTERVAL_S);
        packet.put("INTAKE_POWER (shot feed)", INTAKE_POWER);
        packet.put("FEEDER_POWER (shot feed)", FEEDER_POWER);

        // our PIDF gains (active mode)
        packet.put("shooter1 kP", nectar ? NECTAR_kP : POLLEN_kP);
        packet.put("shooter1 kF", nectar ? NECTAR_kF : POLLEN_kF);
        packet.put("shooter2 kP", nectar ? NECTAR2_kP : POLLEN2_kP);
        packet.put("shooter2 kF", nectar ? NECTAR2_kF : POLLEN2_kF);

        packet.put("loop ms", loopMs);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
