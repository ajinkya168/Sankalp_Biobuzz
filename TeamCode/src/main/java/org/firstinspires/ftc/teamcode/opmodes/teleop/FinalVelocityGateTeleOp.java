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
 * Final Velocity Gate TeleOp - exactly FinalCommandTeleOp, except the shot feed (LB):
 * after the flap opens and SHOOT_FEED_DELAY_S, intake + feeder feed at SHOOT_*_POWER only while
 * the shooter velocity drop (target - velocity) <= SHOOT_VELOCITY_TOL. Otherwise the feeder runs
 * at SHOOT_SLOW_FEEDER_POWER (intake SHOOT_SLOW_INTAKE_POWER) until the shooter recovers.
 *
 * Command based Final TeleOp - same controls as the sankalp27 Shooting TeleOp.
 * Drive goes through the Pedro follower from Constants (mecanum + two wheel + navX).
 * Every number is in final_Globals (live on FTC Dashboard).
 *
 * Gamepad 1
 *   Left stick         - drive (field centric), strafe x StrafeSpeed
 *   Right stick        - turn x turnSpeed
 *   Right stick click  - reset heading (field centric forward)
 *   Right bumper       - field centric <-> robot centric
 *
 *   A           - intake + feeder start / stop. Start: pollen count -> 0, flap CLOSED,
 *                 nectar on the limit switches -> NECTAR, otherwise POLLEN
 *   B           - shooter on / off (TELE_POLLEN_VELOCITY / TELE_NECTAR_VELOCITY), back to POLLEN,
 *                 intake + feeder off
 *   BACK        - POLLEN <-> NECTAR (velocity + gains + hood + adjust servo)
 *   Left bumper - SHOOT: count -> 0, shooter on, flap OPEN, feeder reversed SHOOT_REVERSE_S,
 *                 then intake + feeder feed (SHOOT_*_POWER) not before SHOOT_FEED_DELAY_S,
 *                 only while target - velocity <= SHOOT_VELOCITY_TOL (else slow feed),
 *                 pausing for nectar / adjust servo moves. Again - stop shooting.
 *   X (hold)    - reverse intake + feeder
 *   Dpad left   - stop intake + feeder
 *   Dpad right  - full speed feed (only when stopped), flap OPEN
 *   Dpad up     - servo2 CLOSE
 *   Dpad down   - servo2 OPEN
 *
 * Automatic (FinalSensorThread): pollen count on b1 (FEEDER_STOP_COUNT / MAX_POLLEN),
 * POLLEN / NECTAR follows the piece on the limit switches.
 * Turret held at TELE_TURRET_POS.
 */
@TeleOp(name = "Final Velocity Gate TeleOp", group = "Final")
public class FinalVelocityGateTeleOp extends CommandOpMode {

    private final FinalHardware robot = FinalHardware.getInstance();
    Follower follower;
    FinalIntakeSubsystem intakess;
    FinalShooterSubsystem shooterss;
    FinalTurretSubsystem turretss;
    FinalSensorThread sensors;

    private Gamepad CG1, PG1;

    private boolean startFlag = true;
    private boolean fieldCentric = true;

    // heading (radians) that counts as "forward" for field centric, set by right stick click
    private double headingOffset = 0.0;

    private final ElapsedTime loopTimer = new ElapsedTime();

    // motor current (amps) - each getCurrent() is its own hub read, so not every loop
    private static final String[] MOTOR_NAMES = {"fl", "fr", "bl", "br", "intake", "feeder", "shooter1", "shooter2"};
    private DcMotorEx[] currentMotors;
    private final double[] motorAmps = new double[MOTOR_NAMES.length];
    private final ElapsedTime currentTimer = new ElapsedTime();
    private static final double CURRENT_READ_MS = 100;

    @Override
    public void initialize() {

        CommandScheduler.getInstance().reset();

        robot.init(hardwareMap);
        follower = Constants.create(hardwareMap);
        shooterss = new FinalShooterSubsystem(robot);
        intakess = new FinalIntakeSubsystem(robot, shooterss);
        intakess.velocityGate = true; // shoot only when the shooter is within SHOOT_VELOCITY_TOL
        turretss = new FinalTurretSubsystem(robot);
        sensors = new FinalSensorThread(robot, intakess, shooterss, gamepad1);

        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        currentMotors = new DcMotorEx[MOTOR_NAMES.length];
        for (int i = 0; i < MOTOR_NAMES.length; i++) {
            currentMotors[i] = hardwareMap.get(DcMotorEx.class, MOTOR_NAMES[i]);
        }

        //Gamepads
        CG1 = new Gamepad();
        PG1 = new Gamepad();

        telemetry.addLine("Final Velocity Gate TeleOp ready - POLLEN");
        telemetry.update();
    }

    // stop the sensor thread and every motor however the OpMode ends
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

    @Override
    public void run() {
        double loopMs = loopTimer.milliseconds();
        loopTimer.reset();

        CommandScheduler.getInstance().run();

        // Game Manual 0 - save last loop's gamepad, then read the new one
        PG1.copy(CG1);
        CG1.copy(gamepad1);

        //TODO First loop after start
        if (startFlag) {
            schedule(initSequence(intakess, shooterss));
            sensors.start();
            startFlag = false;
        }

        drive();
        buttons();
        showTelemetry(loopMs);
    }

    private boolean pressed(boolean current, boolean previous) {
        return current && !previous;
    }

    //TODO Drive
    private void drive() {
        if (pressed(CG1.right_bumper, PG1.right_bumper)) fieldCentric = !fieldCentric;

        if (pressed(CG1.right_stick_button, PG1.right_stick_button)) {
            headingOffset = follower.pose().heading();
        }

        double y = deadzone(-CG1.left_stick_y);               // forward
        double x = deadzone(CG1.left_stick_x) * StrafeSpeed;   // strafe right
        double turn = deadzone(CG1.right_stick_x) * turnSpeed; // turn right

        // Pedro: forward +, strafe left +, turn counter-clockwise +
        double heading = fieldCentric ? follower.pose().heading() - headingOffset : 0.0;
        follower.manual(ManualDrive.fieldCentric(y, -x, -turn, heading));
        follower.update();
    }

    //TODO Buttons
    private void buttons() {
        if (pressed(CG1.a, PG1.a)) schedule(intakeToggle(intakess, shooterss));
        if (pressed(CG1.b, PG1.b)) schedule(shooterToggle(intakess, shooterss));
        if (pressed(CG1.back, PG1.back)) schedule(modeToggle(shooterss));
        if (pressed(CG1.left_bumper, PG1.left_bumper)) schedule(shoot(intakess, shooterss, sensors));

        intakess.reversing = CG1.x;

        if (pressed(CG1.dpad_left, PG1.dpad_left)) schedule(intakeStop(intakess, shooterss));
        if (pressed(CG1.dpad_right, PG1.dpad_right)) schedule(fullSpeed(intakess, shooterss));
        if (pressed(CG1.dpad_up, PG1.dpad_up)) schedule(servo2Close(shooterss));
        if (pressed(CG1.dpad_down, PG1.dpad_down)) schedule(servo2Open(shooterss));
    }

    private static double deadzone(double value) {
        return Math.abs(value) < 0.05 ? 0.0 : value;
    }

    //TODO Telemetry
    private void showTelemetry(double loopMs) {
        long now = System.nanoTime();
        telemetry.addData("Loop time (ms)", "%.1f  (%.0f Hz)", loopMs, loopMs > 0 ? 1000 / loopMs : 0);

        telemetry.addData("MODE", shooterss.mode);
        telemetry.addData("Shooter (B)", shooterss.isOn()
                ? (shooterss.atSpeed() ? "ON - AT SPEED" : "ON - spinning up") : "OFF");
        telemetry.addData("  target / velocity / power", "%.0f / %.0f / %.3f",
                shooterss.target, shooterss.velocity, shooterss.power);
        telemetry.addData("Shooting (LB)", intakess.shooting
                ? (intakess.shootReversing(now) ? "REVERSE (start of shot)"
                        : intakess.shootWaiting(now) ? "flap open, waiting"
                        : !shooterss.adjustReady() ? "PAUSED - nectar interval / adjust servo"
                        : intakess.shootSpeedReady() ? "FEEDING" : "SLOW FEED - waiting for shooter speed") : "no");
        double drop = shooterss.target - shooterss.velocity;
        telemetry.addData("  gate drop (target - vel) / tol", "%.0f / %.0f  -> %s", drop, SHOOT_VELOCITY_TOL,
                drop <= SHOOT_VELOCITY_TOL ? "READY (full feed)" : "TOO SLOW (slow feed)");

        String countNote = intakess.pollen.isFull() ? "  FULL - LB to shoot / A to reset"
                : intakess.pollen.isFeederFull() ? "  feeder stopped"
                : intakess.pollen.isFeederStopping() ? "  feeder stopping..." : "";
        telemetry.addData("POLLEN COUNT", "%d / %d%s", intakess.pollen.getCount(), MAX_POLLEN, countNote);
        telemetry.addData("Intake", "%s  (%.2f)", intakess.reversing ? "REVERSE" : intakess.intakeState, intakess.intakePower);
        telemetry.addData("Feeder", "%.2f", intakess.feederPower);

        readMotorCurrents();
        telemetry.addData("Current (A) drive fl / fr / bl / br", "%.2f / %.2f / %.2f / %.2f",
                motorAmps[0], motorAmps[1], motorAmps[2], motorAmps[3]);
        telemetry.addData("Current (A) intake / feeder", "%.2f / %.2f", motorAmps[4], motorAmps[5]);
        telemetry.addData("Current (A) shooter1 / shooter2", "%.2f / %.2f", motorAmps[6], motorAmps[7]);

        telemetry.addData("Flap / Servo2", "%s / %s", shooterss.flapState, shooterss.servo2State);
        telemetry.addData("B1 / B2", "%s / %s", sensors.b1Broken ? "BROKEN" : "clear", sensors.b2Broken ? "BROKEN" : "clear");
        telemetry.addData("Limit 1 / 2", "%s / %s  %s", sensors.limit1Pressed ? "PRESSED" : "released",
                sensors.limit2Pressed ? "PRESSED" : "released", sensors.nectarAtSwitch ? "(NECTAR on switches)" : "");
        telemetry.addData("Turret pos", "%.3f", turretss.sentPos);

        telemetry.addData("Drive", fieldCentric ? "FIELD-centric (RB to switch)" : "ROBOT-centric (RB to switch)");
        telemetry.addData("Heading", "%.1f",
                Math.toDegrees(Angle.normalizeSigned(follower.pose().heading() - headingOffset)));
        telemetry.addData("Pose", follower.pose());
        telemetry.addData("Sensor thread (Hz)", "%.0f", sensors.hz);
        telemetry.addLine("A intake | B shooter | BACK pollen/nectar | LB shoot | X reverse | dpad L stop | dpad R full speed | dpad U/D servo2 | RB field/robot | R-stick zero heading");
        telemetry.update();

        sendGraphs(loopMs, now);
    }

    private void readMotorCurrents() {
        if (currentTimer.milliseconds() < CURRENT_READ_MS) return;
        currentTimer.reset();
        for (int i = 0; i < currentMotors.length; i++) motorAmps[i] = currentMotors[i].getCurrent(CurrentUnit.AMPS);
    }

    /*
     * FTC Dashboard graphs: numbers only. Suggested graphs:
     *   velocity : shooter target, shooter velocity
     *   events   : feeding, paused, reversing, nectar mode, limit 1 / 2, nectar at switch
     *   tuning   : the final_Globals values below (flat lines - shows when you change one)
     */
    private void sendGraphs(double loopMs, long now) {
        boolean reversingStart = intakess.shootReversing(now);
        boolean feeding = intakess.shootFeeding(now);
        boolean paused = intakess.shooting && !reversingStart && !feeding;
        boolean slowFeed = feeding && !intakess.shootSpeedReady();
        boolean nectar = shooterss.mode == FinalShooterSubsystem.MODE.NECTAR;

        TelemetryPacket packet = new TelemetryPacket();
        // velocity
        packet.put("shooter target", shooterss.target);
        packet.put("shooter velocity", shooterss.velocity);
        packet.put("shooter error", shooterss.target - shooterss.velocity);
        packet.put("shooter power", shooterss.power);
        packet.put("battery voltage", shooterss.voltage);
        // events (0 / 1)
        packet.put("shooting", intakess.shooting ? 1 : 0);
        packet.put("feeding", feeding && !slowFeed ? 1 : 0);
        packet.put("slow feed (waiting for speed)", slowFeed ? 1 : 0);
        // velocity gate - graph "gate drop" against "gate tolerance": above the line = slow feed
        packet.put("gate drop (target - velocity)", shooterss.target - shooterss.velocity);
        packet.put("gate tolerance", SHOOT_VELOCITY_TOL);
        packet.put("gate speed ready", intakess.shootSpeedReady() ? 1 : 0);
        packet.put("paused", paused ? 1 : 0);
        packet.put("reversing (start of shot)", reversingStart ? 1 : 0);
        packet.put("nectar mode", nectar ? 1 : 0);
        packet.put("nectar at switch", sensors.nectarAtSwitch ? 1 : 0);
        packet.put("limit 1", sensors.limit1Pressed ? 1 : 0);
        packet.put("limit 2", sensors.limit2Pressed ? 1 : 0);
        packet.put("pollen count", intakess.pollen.getCount());
        packet.put("adjust position", robot.adjust.getPosition());
        packet.put("intake power", intakess.intakePower);
        packet.put("feeder power", intakess.feederPower);
        // motor current (A), refreshed every CURRENT_READ_MS
        for (int i = 0; i < MOTOR_NAMES.length; i++) packet.put("current " + MOTOR_NAMES[i], motorAmps[i]);
        // tuning values (final_Globals)
        packet.put("TELE_POLLEN_VELOCITY", TELE_POLLEN_VELOCITY);
        packet.put("TELE_NECTAR_VELOCITY", TELE_NECTAR_VELOCITY);
        packet.put("SHOOT_FEED_DELAY_S", SHOOT_FEED_DELAY_S);
        packet.put("SHOOT_REVERSE_S", SHOOT_REVERSE_S);
        packet.put("SHOOT_REVERSE_POWER", SHOOT_REVERSE_POWER);
        packet.put("ADJUST_MOVE_S", ADJUST_MOVE_S);
        packet.put("NECTAR_INTERVAL_S", NECTAR_INTERVAL_S);
        packet.put("NECTAR_CLEAR_S", NECTAR_CLEAR_S);
        packet.put("NECTAR_DETECT_MS", NECTAR_DETECT_MS);
        packet.put("SHOOT_INTAKE_POWER", SHOOT_INTAKE_POWER);
        packet.put("SHOOT_FEEDER_POWER", SHOOT_FEEDER_POWER);
        packet.put("SHOOT_VELOCITY_TOL", SHOOT_VELOCITY_TOL);
        packet.put("SHOOT_SLOW_FEEDER_POWER", SHOOT_SLOW_FEEDER_POWER);
        packet.put("SHOOT_SLOW_INTAKE_POWER", SHOOT_SLOW_INTAKE_POWER);
        packet.put("SHOOTER1_HUB_kP", SHOOTER1_HUB_kP);
        packet.put("SHOOTER2_HUB_kP", SHOOTER2_HUB_kP);
        packet.put("SHOOTER_HUB_kI", SHOOTER_HUB_kI);
        packet.put("SHOOTER_HUB_kD", SHOOTER_HUB_kD);
        packet.put("SHOOTER1_HUB_kF", SHOOTER1_HUB_kF);
        packet.put("SHOOTER2_HUB_kF", SHOOTER2_HUB_kF);
        packet.put("loop ms", loopMs);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
