package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.hdware.final_Globals.*;
import static org.firstinspires.ftc.teamcode.sequences.FinalSequences.fullSpeed;
import static org.firstinspires.ftc.teamcode.sequences.FinalSequences.initSequence;
import static org.firstinspires.ftc.teamcode.sequences.FinalSequences.intakeStop;
import static org.firstinspires.ftc.teamcode.sequences.FinalSequences.intakeToggle;
import static org.firstinspires.ftc.teamcode.sequences.FinalSequences.modeToggle;
import static org.firstinspires.ftc.teamcode.sequences.FinalSequences.servo2Close;
import static org.firstinspires.ftc.teamcode.sequences.FinalSequences.servo2Open;
import static org.firstinspires.ftc.teamcode.sequences.FinalSequences.shoot;
import static org.firstinspires.ftc.teamcode.sequences.FinalSequences.shooterToggle;
import static org.firstinspires.ftc.teamcode.sequences.FinalSequences.stopShooting;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.utils.Angle;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hdware.FinalHardware;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.FinalIntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FinalSensorThread;
import org.firstinspires.ftc.teamcode.subsystems.FinalShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FinalTurretSubsystem;

/*
 * ============================== FINAL COMMAND TELEOP ==============================
 *
 * Command based TeleOp for the Final robot.
 *   - Drive:   Pedro follower from Constants (mecanum + two wheel odometry + navX)
 *   - Shooter: OUR velocity PIDF (FinalShooterSubsystem(robot, true)), gains tuned in
 *              "FINAL: Shooter Velocity PIDF Test"
 *   - Numbers: every tunable value is in final_Globals (live on FTC Dashboard)
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
 *     BACK               POLLEN <-> NECTAR (velocity + gains + hood + adjust servo)
 *     Left bumper        SHOOT: shooter on, flap OPEN, feeder reverses SHOOT_REVERSE_S,
 *                        then feeds after SHOOT_FEED_DELAY_S.
 *                        Ends by itself after SHOOT_FLAP_CLOSE_S (flap closes), or press LB again.
 *     Dpad up / down     servo2 CLOSE / OPEN
 *
 * --------------------------------- AUTOMATIC ---------------------------------
 *   FinalSensorThread counts pollen on beam b1 (FEEDER_STOP_COUNT / MAX_POLLEN) and
 *   switches POLLEN / NECTAR from the limit switches. Turret is held at TELE_TURRET_POS.
 */
@TeleOp(name = "Final Command TeleOp", group = "Final")
public class FinalCommandTeleOp extends CommandOpMode {

    // ------------------------------ hardware + subsystems ------------------------------
    private final FinalHardware robot = FinalHardware.getInstance();
    private Follower follower;
    private FinalIntakeSubsystem intakess;
    private FinalShooterSubsystem shooterss;
    private FinalTurretSubsystem turretss;
    private FinalSensorThread sensors;

    // ------------------------------ gamepad (current + previous loop) ------------------------------
    private Gamepad CG1, PG1;

    // ------------------------------ drive state ------------------------------
    private boolean fieldCentric = true;
    private double headingOffset = 0.0;     // radians that count as "forward", set by right stick click

    // ------------------------------ loop state ------------------------------
    private boolean startFlag = true;       // true until the first loop after START has run
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
    public void initialize() {
        CommandScheduler.getInstance().reset();

        // hardware + subsystems
        robot.init(hardwareMap);
        follower = Constants.create(hardwareMap);
        shooterss = new FinalShooterSubsystem(robot, true);   // true = our PIDF
        intakess = new FinalIntakeSubsystem(robot, shooterss);
        intakess.shootAtIntakePowers = true;                  // shot feed at INTAKE_POWER / FEEDER_POWER
        turretss = new FinalTurretSubsystem(robot);
        sensors = new FinalSensorThread(robot, intakess, shooterss, gamepad1);

        // bulk reads: one hub read per loop for all encoders / sensors
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        currentMotors = new DcMotorEx[MOTOR_NAMES.length];
        for (int i = 0; i < MOTOR_NAMES.length; i++) {
            currentMotors[i] = hardwareMap.get(DcMotorEx.class, MOTOR_NAMES[i]);
        }

        CG1 = new Gamepad();
        PG1 = new Gamepad();

        telemetry.addLine("Final Command TeleOp ready - POLLEN");
        telemetry.update();
    }

    /** Wraps the normal OpMode so the sensor thread and every motor stop however the OpMode ends. */
    @Override
    public void runOpMode() throws InterruptedException {
        try {
            super.runOpMode();
        } finally {
            if (sensors != null) sensors.stop();
            if (intakess != null) intakess.stop();
            if (shooterss != null) shooterss.stop();
            if (follower != null) follower.stop();
        }
    }

    // =====================================================================================
    //                                    MAIN LOOP
    // =====================================================================================

    @Override
    public void run() {
        double loopMs = loopTimer.milliseconds();
        loopTimer.reset();

        // runs every subsystem's periodic() (motor powers) + any scheduled commands
        CommandScheduler.getInstance().run();

        // save last loop's gamepad, then read the new one (for "just pressed" checks)
        PG1.copy(CG1);
        CG1.copy(gamepad1);

        // first loop after START: set the robot up and start the sensor thread
        if (startFlag) {
            schedule(initSequence(intakess, shooterss));
            sensors.start();
            startFlag = false;
        }

        drive();
        buttons();
        autoCloseFlap();
        showTelemetry(loopMs);
    }

    // =====================================================================================
    //                                      DRIVE
    // =====================================================================================

    private void drive() {
        if (pressed(CG1.right_bumper, PG1.right_bumper)) fieldCentric = !fieldCentric;
        if (pressed(CG1.right_stick_button, PG1.right_stick_button)) {
            headingOffset = follower.pose().heading();
        }

        // our sticks: forward +, strafe right +, turn right +
        double y = deadzone(-CG1.left_stick_y);
        double x = deadzone(CG1.left_stick_x) * StrafeSpeed;
        double turn = deadzone(CG1.right_stick_x) * turnSpeed;

        // Pedro wants: forward +, strafe LEFT +, turn COUNTER-clockwise + (so x and turn are flipped)
        double heading = fieldCentric ? follower.pose().heading() - headingOffset : 0.0;
        follower.manual(ManualDrive.fieldCentric(y, -x, -turn, heading));
        follower.update();
    }

    private static double deadzone(double value) {
        return Math.abs(value) < 0.05 ? 0.0 : value;
    }

    // =====================================================================================
    //                                     BUTTONS
    // =====================================================================================

    private void buttons() {
        // intake
        if (pressed(CG1.a, PG1.a)) schedule(intakeToggle(intakess, shooterss));
        intakess.reversing = CG1.x;                                         // hold
        if (pressed(CG1.dpad_left, PG1.dpad_left)) schedule(intakeStop(intakess, shooterss));
        if (pressed(CG1.dpad_right, PG1.dpad_right)) schedule(fullSpeed(intakess, shooterss));

        // shooter
        if (pressed(CG1.b, PG1.b)) schedule(shooterToggle(intakess, shooterss));
        if (pressed(CG1.back, PG1.back)) schedule(modeToggle(shooterss));
        if (pressed(CG1.left_bumper, PG1.left_bumper)) schedule(shoot(intakess, shooterss, sensors));

        // servo2
        if (pressed(CG1.dpad_up, PG1.dpad_up)) schedule(servo2Close(shooterss));
        if (pressed(CG1.dpad_down, PG1.dpad_down)) schedule(servo2Open(shooterss));
    }

    /** True only on the loop the button goes from up to down. */
    private boolean pressed(boolean current, boolean previous) {
        return current && !previous;
    }

    /**
     * LB auto close: SHOOT_FLAP_CLOSE_S after LB started a shot, end it
     * (feeding stops + flap closes), same as pressing LB again.
     * SHOOT_FLAP_CLOSE_S <= 0 turns this off.
     */
    private void autoCloseFlap() {
        if (SHOOT_FLAP_CLOSE_S <= 0 || !intakess.shooting) return;
        if (intakess.shootElapsedS(System.nanoTime()) >= SHOOT_FLAP_CLOSE_S) {
            schedule(stopShooting(intakess, shooterss));
        }
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
        telemetry.addData("MODE", shooterss.mode);
        telemetry.addData("Shooter (B)", shooterss.isOn()
                ? (shooterss.atSpeed() ? "ON - AT SPEED" : "ON - spinning up") : "OFF");
        telemetry.addData("  target / velocity / power 1 / power 2", "%.0f / %.0f / %.3f / %.3f",
                shooterss.target, shooterss.velocity, shooterss.power, shooterss.power2);
        telemetry.addData("Shooting (LB)", shootingStatus(now));

        // intake + pollen count
        telemetry.addData("POLLEN COUNT", "%d / %d%s", intakess.pollen.getCount(), MAX_POLLEN, pollenNote());
        telemetry.addData("Intake", "%s  (%.2f)", intakess.reversing ? "REVERSE" : intakess.intakeState, intakess.intakePower);
        telemetry.addData("Feeder", "%.2f", intakess.feederPower);

        // motor currents
        telemetry.addData("Current (A) drive fl / fr / bl / br", "%.2f / %.2f / %.2f / %.2f",
                motorAmps[0], motorAmps[1], motorAmps[2], motorAmps[3]);
        telemetry.addData("Current (A) intake / feeder", "%.2f / %.2f", motorAmps[4], motorAmps[5]);
        telemetry.addData("Current (A) shooter1 / shooter2", "%.2f / %.2f", motorAmps[6], motorAmps[7]);

        // servos + sensors
        telemetry.addData("Flap / Servo2", "%s / %s", shooterss.flapState, shooterss.servo2State);
        telemetry.addData("B1 / B2", "%s / %s", sensors.b1Broken ? "BROKEN" : "clear", sensors.b2Broken ? "BROKEN" : "clear");
        telemetry.addData("Limit 1 / 2", "%s / %s  %s", sensors.limit1Pressed ? "PRESSED" : "released",
                sensors.limit2Pressed ? "PRESSED" : "released", sensors.nectarAtSwitch ? "(NECTAR on switches)" : "");
        telemetry.addData("Turret pos", "%.3f", turretss.sentPos);

        // drive
        telemetry.addData("Drive", fieldCentric ? "FIELD-centric (RB to switch)" : "ROBOT-centric (RB to switch)");
        telemetry.addData("Heading", "%.1f",
                Math.toDegrees(Angle.normalizeSigned(follower.pose().heading() - headingOffset)));
        telemetry.addData("Pose", follower.pose());
        telemetry.addData("Sensor thread (Hz)", "%.0f", sensors.hz);

        telemetry.addLine("A intake | B shooter | BACK pollen/nectar | LB shoot | X reverse | dpad L stop | dpad R full speed | dpad U/D servo2 | RB field/robot | R-stick zero heading");
        telemetry.update();

        sendGraphs(loopMs, now);
    }

    /** What the LB shot is doing right now, plus time until the flap auto closes. */
    private String shootingStatus(long now) {
        if (!intakess.shooting) return "no";
        String step = intakess.shootReversing(now) ? "REVERSE (start of shot)"
                : intakess.shootWaiting(now) ? "flap open, waiting"
                : shooterss.adjustReady() ? "FEEDING"
                : "PAUSED - nectar interval / adjust servo";
        if (SHOOT_FLAP_CLOSE_S <= 0) return step;
        double left = Math.max(0, SHOOT_FLAP_CLOSE_S - intakess.shootElapsedS(now));
        return String.format("%s  (flap closes in %.1f s)", step, left);
    }

    /** Extra note after the pollen count (full / feeder stopped / stopping). */
    private String pollenNote() {
        if (intakess.pollen.isFull()) return "  FULL - LB to shoot / A to reset";
        if (intakess.pollen.isFeederFull()) return "  feeder stopped";
        if (intakess.pollen.isFeederStopping()) return "  feeder stopping...";
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
     * Numbers only (true/false sent as 1/0 so they graph). Suggested graphs:
     *   velocity : shooter target, shooter velocity
     *   events   : shooting, feeding, paused, reversing, nectar mode, limit 1 / 2
     *   tuning   : the final_Globals values (flat lines - shows when you change one)
     */
    private void sendGraphs(double loopMs, long now) {
        boolean reversingStart = intakess.shootReversing(now);
        boolean feeding = intakess.shootFeeding(now);
        boolean paused = intakess.shooting && !reversingStart && !feeding;
        boolean nectar = shooterss.mode == FinalShooterSubsystem.MODE.NECTAR;

        TelemetryPacket packet = new TelemetryPacket();

        // shooter velocity
        packet.put("shooter target", shooterss.target);
        packet.put("shooter velocity", shooterss.velocity);
        packet.put("shooter error", shooterss.target - shooterss.velocity);
        packet.put("shooter1 velocity", robot.getShooter1Velocity());
        packet.put("shooter2 velocity", robot.getShooter2Velocity());
        packet.put("shooter1 power", shooterss.power);
        packet.put("shooter2 power", shooterss.power2);
        packet.put("shooter1 P term", shooterss.pidf1().pTerm);
        packet.put("shooter2 P term", shooterss.pidf2().pTerm);
        packet.put("battery voltage", shooterss.voltage);

        // events (0 / 1)
        packet.put("shooting", intakess.shooting ? 1 : 0);
        packet.put("shot time (s)", intakess.shootElapsedS(now));
        packet.put("feeding", feeding ? 1 : 0);
        packet.put("paused", paused ? 1 : 0);
        packet.put("reversing (start of shot)", reversingStart ? 1 : 0);
        packet.put("flap open", shooterss.flapState == FinalShooterSubsystem.FLAP.OPEN ? 1 : 0);
        packet.put("nectar mode", nectar ? 1 : 0);
        packet.put("nectar at switch", sensors.nectarAtSwitch ? 1 : 0);
        packet.put("limit 1", sensors.limit1Pressed ? 1 : 0);
        packet.put("limit 2", sensors.limit2Pressed ? 1 : 0);

        // intake / feeder / adjust
        packet.put("pollen count", intakess.pollen.getCount());
        packet.put("intake power", intakess.intakePower);
        packet.put("feeder power", intakess.feederPower);
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
        packet.put("NECTAR_CLEAR_S", NECTAR_CLEAR_S);
        packet.put("NECTAR_DETECT_MS", NECTAR_DETECT_MS);
        packet.put("INTAKE_POWER (shot feed)", INTAKE_POWER);
        packet.put("FEEDER_POWER (shot feed)", FEEDER_POWER);

        // our PIDF gains (active mode)
        packet.put("shooter1 kP", nectar ? NECTAR_kP : POLLEN_kP);
        packet.put("shooter1 kF", nectar ? NECTAR_kF : POLLEN_kF);
        packet.put("shooter2 kP", nectar ? NECTAR2_kP : POLLEN2_kP);
        packet.put("shooter2 kF", nectar ? NECTAR2_kF : POLLEN2_kF);
        packet.put("kI (shared)", nectar ? NECTAR_kI : POLLEN_kI);
        packet.put("kD (shared)", nectar ? NECTAR_kD : POLLEN_kD);

        packet.put("loop ms", loopMs);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
