package org.firstinspires.ftc.teamcode.test;

import static org.firstinspires.ftc.teamcode.hdware.final_Globals.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hdware.FinalHardware;
import org.firstinspires.ftc.teamcode.hdware.VelocityPIDF;

/*
 * Final Shooter Test (same as the sankalp27 Shooter Test). Velocity held by VelocityPIDF with
 * separate gains + target for POLLEN and NECTAR, plus hood + adjust servo. Values in
 * final_Globals (SHOOTER_* / POLLEN_k* / NECTAR_k* / HOOD_* / ADJUST_*) - tune them live
 * on FTC Dashboard and watch the graph.
 *
 * Get values: X / B jumps hood + adjust to that mode's saved position, the controls below
 * change the current mode's values (HOOD_POLLEN_POS / HOOD_NECTAR_POS / ADJUST_POLLEN_POS /
 * ADJUST_NECTAR_POS), telemetry shows them - copy them into final_Globals once the shots land.
 *
 *   A             - shooter on / off
 *   X             - POLLEN mode (SHOOTER_POLLEN_VELOCITY + POLLEN_k*)
 *   B             - NECTAR mode (SHOOTER_NECTAR_VELOCITY + NECTAR_k*)
 *   back          - quick switch POLLEN <-> NECTAR
 *   right / left bumper - target velocity of the current mode +/- SHOOTER_VELOCITY_STEP
 *   dpad up / down      - hood +/- HOOD_STEP
 *   dpad right / left   - hood +/- HOOD_FINE_STEP
 *   Y             - hood back to HOOD_INIT_POS
 *   hold left trigger + dpad up / down    - adjust servo +/- ADJUST_STEP
 *   hold left trigger + dpad right / left - adjust servo +/- ADJUST_FINE_STEP
 *   hold left trigger + Y                 - adjust servo back to ADJUST_INIT_POS
 *   left stick up / down - adjust servo, up to ADJUST_SPEED per second
 *   left stick button    - adjust servo back to ADJUST_INIT_POS
 *   right trigger (hold) - intake + feeder, to push pieces in and see the velocity recover
 *   right stick button   - intake + feeder on / off (toggle)
 *   flap stays at flapOpen the whole OpMode
 *
 * Tuning order (per mode): kP = kI = kD = 0, raise kF until the velocity is just under the
 * target, then raise kP until it reaches the target quickly without oscillating, then a
 * little kD if it overshoots. kI only if it settles below the target. Both encoders must
 * read positive while spinning - if one is negative, flip SHOOTER*_ENCODER_REVERSED.
 */
@TeleOp(name = "FINAL: Shooter Test", group = "Final Test")
public class FinalShooterTest extends LinearOpMode {

    private final FinalHardware robot = FinalHardware.getInstance();
    private static final double SLOW_READ_MS = 100; // current + battery voltage are single hub reads - not every loop

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap);
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        double hoodPos = robot.setHood(HOOD_INIT_POS);
        double adjustPos = robot.setAdjust(ADJUST_INIT_POS);
        robot.setShooterPower(0);
        robot.setTurretPosition(TURRET_INIT_POS);
        robot.flap.setPosition(flapOpen); // flap open all the time in this test
        VelocityPIDF pidf = new VelocityPIDF();

        while (opModeInInit()) {
            telemetry.addLine("Final Shooter Test - A = on/off, X = POLLEN, B = NECTAR, bumpers = velocity, dpad = hood, left stick = adjust");
            telemetry.update();
        }
        if (isStopRequested()) return;

        boolean shooterOn = false;
        boolean pollenMode = true;
        boolean aPrev = false, xPrev = false, bPrev = false, rbPrev = false, lbPrev = false;
        boolean upPrev = false, downPrev = false, rightPrev = false, leftPrev = false;
        boolean feedOn = false, rsPrev = false, backPrev = false;
        double voltage = robot.voltageSensor.getVoltage();
        double amps1 = 0, amps2 = 0;
        double peakError = 0;
        ElapsedTime slowTimer = new ElapsedTime();
        ElapsedTime loopTimer = new ElapsedTime();

        while (opModeIsActive()) {
            double loopMs = loopTimer.milliseconds();
            loopTimer.reset();

            //TODO buttons
            if (gamepad1.a && !aPrev) {
                shooterOn = !shooterOn;
                pidf.reset();
                peakError = 0;
            }
            if (gamepad1.x && !xPrev && !pollenMode) {
                pollenMode = true;
                pidf.reset();
                peakError = 0;
            }
            if (gamepad1.b && !bPrev && pollenMode) {
                pollenMode = false;
                pidf.reset();
                peakError = 0;
            }
            if (gamepad1.back && !backPrev) {
                pollenMode = !pollenMode;
                pidf.reset();
                peakError = 0;
            }
            backPrev = gamepad1.back;
            if (gamepad1.right_bumper && !rbPrev) changeTarget(pollenMode, SHOOTER_VELOCITY_STEP);
            if (gamepad1.left_bumper && !lbPrev) changeTarget(pollenMode, -SHOOTER_VELOCITY_STEP);

            //TODO hood + adjust get values - the current mode's saved positions (also live on Dashboard)
            hoodPos = pollenMode ? HOOD_POLLEN_POS : HOOD_NECTAR_POS;
            adjustPos = pollenMode ? ADJUST_POLLEN_POS : ADJUST_NECTAR_POS;
            boolean adjustButtons = gamepad1.left_trigger > 0.1; // hold LT: dpad / Y work the adjust servo instead of the hood
            if (adjustButtons) {
                if (gamepad1.dpad_up && !upPrev) adjustPos += ADJUST_STEP;
                if (gamepad1.dpad_down && !downPrev) adjustPos -= ADJUST_STEP;
                if (gamepad1.dpad_right && !rightPrev) adjustPos += ADJUST_FINE_STEP;
                if (gamepad1.dpad_left && !leftPrev) adjustPos -= ADJUST_FINE_STEP;
                if (gamepad1.y) adjustPos = ADJUST_INIT_POS;
            } else {
                if (gamepad1.dpad_up && !upPrev) hoodPos += HOOD_STEP;
                if (gamepad1.dpad_down && !downPrev) hoodPos -= HOOD_STEP;
                if (gamepad1.dpad_right && !rightPrev) hoodPos += HOOD_FINE_STEP;
                if (gamepad1.dpad_left && !leftPrev) hoodPos -= HOOD_FINE_STEP;
                if (gamepad1.y) hoodPos = HOOD_INIT_POS;
            }
            double stick = -gamepad1.left_stick_y;
            if (Math.abs(stick) > 0.05) adjustPos += stick * ADJUST_SPEED * loopMs / 1000.0;
            if (gamepad1.left_stick_button) adjustPos = ADJUST_INIT_POS;
            hoodPos = robot.setHood(hoodPos);
            adjustPos = robot.setAdjust(adjustPos);
            if (pollenMode) {
                HOOD_POLLEN_POS = hoodPos;
                ADJUST_POLLEN_POS = adjustPos;
            } else {
                HOOD_NECTAR_POS = hoodPos;
                ADJUST_NECTAR_POS = adjustPos;
            }

            aPrev = gamepad1.a;
            xPrev = gamepad1.x;
            bPrev = gamepad1.b;
            rbPrev = gamepad1.right_bumper;
            lbPrev = gamepad1.left_bumper;
            upPrev = gamepad1.dpad_up;
            downPrev = gamepad1.dpad_down;
            rightPrev = gamepad1.dpad_right;
            leftPrev = gamepad1.dpad_left;
            if (gamepad1.right_stick_button && !rsPrev) feedOn = !feedOn;
            rsPrev = gamepad1.right_stick_button;

            //TODO slow reads
            if (slowTimer.milliseconds() >= SLOW_READ_MS) {
                slowTimer.reset();
                voltage = robot.voltageSensor.getVoltage();
                amps1 = robot.shooter1.getCurrent(CurrentUnit.AMPS);
                amps2 = robot.shooter2.getCurrent(CurrentUnit.AMPS);
            }

            //TODO shooter PIDF
            double target = pollenMode ? SHOOTER_POLLEN_VELOCITY : SHOOTER_NECTAR_VELOCITY;
            double v1 = robot.getShooter1Velocity();
            double v2 = robot.getShooter2Velocity();
            double velocity = SHOOTER_USE_BOTH_ENCODERS ? (v1 + v2) / 2 : v1;

            double power = 0;
            if (shooterOn) {
                power = pollenMode
                        ? pidf.update(target, velocity, POLLEN_kP, POLLEN_kI, POLLEN_kD, POLLEN_kF, SHOOTER_I_MAX)
                        : pidf.update(target, velocity, NECTAR_kP, NECTAR_kI, NECTAR_kD, NECTAR_kF, SHOOTER_I_MAX);
                if (SHOOTER_VOLTAGE_COMP && voltage > 1) power *= NOMINAL_VOLTAGE / voltage;
                power = Range.clip(power, 0, 1); // never drive the flywheel backwards
                peakError = Math.max(peakError, Math.abs(target - velocity));
            }
            robot.setShooterPower(power);
            boolean atSpeed = shooterOn && Math.abs(target - velocity) <= SHOOTER_READY_TOL;

            //TODO feed (right trigger hold, or right stick button toggle) - flap held open
            boolean feeding = feedOn || gamepad1.right_trigger > 0.5;
            robot.flap.setPosition(flapOpen);
            robot.intake.setPower(feeding ? -INTAKE_POWER : 0);
            robot.feeder.setPower(feeding ? FEEDER_POWER : 0);

            //TODO telemetry (numbers graph on FTC Dashboard)
            telemetry.addData("Mode", pollenMode ? "POLLEN" : "NECTAR");
            telemetry.addData("Shooter", shooterOn ? (atSpeed ? "ON - AT SPEED" : "ON - spinning up") : "OFF");
            telemetry.addData("target", target);
            telemetry.addData("velocity", velocity);
            telemetry.addData("error", shooterOn ? target - velocity : 0);
            telemetry.addData("peak |error| (since mode/on)", "%.0f", peakError);
            telemetry.addData("shooter1 / shooter2 vel", "%.0f / %.0f", v1, v2);
            telemetry.addData("power", "%.3f", power);
            telemetry.addData("P / I / D / F", "%.3f / %.3f / %.3f / %.3f", pidf.pTerm, pidf.iTerm, pidf.dTerm, pidf.fTerm);
            telemetry.addData("gains kP / kI / kD / kF", pollenMode
                    ? String.format("%.5f / %.5f / %.5f / %.5f", POLLEN_kP, POLLEN_kI, POLLEN_kD, POLLEN_kF)
                    : String.format("%.5f / %.5f / %.5f / %.5f", NECTAR_kP, NECTAR_kI, NECTAR_kD, NECTAR_kF));
            telemetry.addData("shooter1 / shooter2 (A)", "%.2f / %.2f", amps1, amps2);
            telemetry.addData("Battery (V)", "%.2f", voltage);
            telemetry.addData("Hood pos (hood1 / hood2)", "%.4f / %.4f", robot.hood1.getPosition(), robot.hood2.getPosition());
            telemetry.addData("Adjust pos", "%.4f", adjustPos);
            telemetry.addData("HOOD_POLLEN_POS / HOOD_NECTAR_POS", "%.4f / %.4f", HOOD_POLLEN_POS, HOOD_NECTAR_POS);
            telemetry.addData("ADJUST_POLLEN_POS / ADJUST_NECTAR_POS", "%.4f / %.4f", ADJUST_POLLEN_POS, ADJUST_NECTAR_POS);
            telemetry.addData("Feeding (RT hold / RS toggle)", feeding ? (feedOn ? "ON (toggle)" : "ON (RT)") : "OFF");
            telemetry.addData("Loop time (ms)", "%.1f", loopMs);
            telemetry.addLine("A on/off | X pollen | B nectar | BACK switch | RB/LB velocity +/- | dpad U/D hood | dpad R/L hood fine | Y hood init | LT + dpad/Y adjust | L stick adjust | RT feed");
            telemetry.update();
        }

        robot.setShooterPower(0);
        robot.intake.setPower(0);
        robot.feeder.setPower(0);
    }

    private static void changeTarget(boolean pollenMode, double delta) {
        if (pollenMode) SHOOTER_POLLEN_VELOCITY = Math.max(0, SHOOTER_POLLEN_VELOCITY + delta);
        else SHOOTER_NECTAR_VELOCITY = Math.max(0, SHOOTER_NECTAR_VELOCITY + delta);
    }
}
