package org.firstinspires.ftc.teamcode.test;

import static org.firstinspires.ftc.teamcode.hdware.final_Globals.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hdware.FinalHardware;

/*
 * Final Turret Increment Test - step the turret by hand to find servo positions
 * (e.g. RedCloseAuto TURRET_PRELOAD / TURRET_FLOWER, AUTO_TURRET_POS, TELE_TURRET_POS).
 *
 * Both servos move together: turret1 = pos, turret2 = 1 - pos + TURRET2_TRIM (mirrored).
 * 0.5 = turret facing robot-forward (0 deg).
 *
 *   dpad up / down       +/- 0.01   (fine)
 *   dpad right / left    +/- 0.05   (coarse)
 *   right / left bumper  +/- 0.002  (extra fine)
 *   A                    back to 0.5 (forward)
 *   X                    save as PRELOAD value  (shown on telemetry)
 *   B                    save as FLOWER value   (shown on telemetry)
 *   Y                    save as OTHER value    (shown on telemetry)
 *   gamepad2 dpad up/down  TURRET2_TRIM +/- 0.002 (if the two servos fight / buzz)
 *
 * Place the robot where it shoots, step the turret until it points at the hive,
 * then copy the position into the auto.
 */
@TeleOp(name = "FINAL: Turret Increment Test", group = "Final Test")
public class FinalTurretIncrementTest extends LinearOpMode {

    private static final double CENTER = 0.5;
    private static final double FINE = 0.01;
    private static final double COARSE = 0.05;
    private static final double EXTRA_FINE = 0.002;
    private static final double TRIM_STEP = 0.002;

    private final FinalHardware robot = FinalHardware.getInstance();

    // previous button states (edge detection)
    private boolean up, down, left, right, lb, rb, a, x, b, y, up2, down2;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(hardwareMap);

        double pos = CENTER;
        double sent = robot.setTurretPosition(pos);
        double preload = Double.NaN, flower = Double.NaN, other = Double.NaN;

        while (opModeInInit()) {
            telemetry.addLine("Turret Increment Test - turret moves to 0.5 (forward) now");
            telemetry.addData("Sent", "%.3f", sent);
            telemetry.addLine("Press START");
            telemetry.update();
        }
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // ---------- position steps ----------
            if (pressed(gamepad1.dpad_up, up))      pos += FINE;
            if (pressed(gamepad1.dpad_down, down))  pos -= FINE;
            if (pressed(gamepad1.dpad_right, right)) pos += COARSE;
            if (pressed(gamepad1.dpad_left, left))  pos -= COARSE;
            if (pressed(gamepad1.right_bumper, rb)) pos += EXTRA_FINE;
            if (pressed(gamepad1.left_bumper, lb))  pos -= EXTRA_FINE;
            if (pressed(gamepad1.a, a))             pos = CENTER;

            // keep the wanted value inside the soft limits, so it doesn't "wind up" past them
            pos = Math.max(TURRET_MIN_POS, Math.min(TURRET_MAX_POS, pos));
            pos = Math.round(pos * 1000.0) / 1000.0;   // no floating point junk like 0.4300000001

            // ---------- trim for turret2 ----------
            if (pressed(gamepad2.dpad_up, up2))     TURRET2_TRIM += TRIM_STEP;
            if (pressed(gamepad2.dpad_down, down2)) TURRET2_TRIM -= TRIM_STEP;

            sent = robot.setTurretPosition(pos);

            // ---------- save values ----------
            if (pressed(gamepad1.x, x)) preload = sent;
            if (pressed(gamepad1.b, b)) flower = sent;
            if (pressed(gamepad1.y, y)) other = sent;

            remember();

            // ---------- telemetry ----------
            telemetry.addData("Turret position", "%.3f", sent);
            telemetry.addData("Approx angle (deg, CCW +)", "%.1f", FinalHardware.turretPositionToAngle(sent));
            telemetry.addData("turret1 / turret2", "%.3f / %.3f",
                    robot.turret1.getPosition(), robot.turret2.getPosition());
            telemetry.addData("TURRET2_TRIM", "%.3f", TURRET2_TRIM);
            telemetry.addData("Soft limits", "%.2f - %.2f", TURRET_MIN_POS, TURRET_MAX_POS);
            telemetry.addLine();
            telemetry.addData("Saved PRELOAD (X)", fmt(preload));
            telemetry.addData("Saved FLOWER  (B)", fmt(flower));
            telemetry.addData("Saved OTHER   (Y)", fmt(other));
            telemetry.addLine();
            telemetry.addLine("dpad U/D +-0.01 | dpad R/L +-0.05 | RB/LB +-0.002 | A = 0.5");
            telemetry.addLine("X/B/Y save | gamepad2 dpad U/D = turret2 trim");
            telemetry.update();
        }
    }

    private static boolean pressed(boolean now, boolean before) {
        return now && !before;
    }

    private void remember() {
        up = gamepad1.dpad_up;
        down = gamepad1.dpad_down;
        left = gamepad1.dpad_left;
        right = gamepad1.dpad_right;
        lb = gamepad1.left_bumper;
        rb = gamepad1.right_bumper;
        a = gamepad1.a;
        x = gamepad1.x;
        b = gamepad1.b;
        y = gamepad1.y;
        up2 = gamepad2.dpad_up;
        down2 = gamepad2.dpad_down;
    }

    private static String fmt(double v) {
        return Double.isNaN(v) ? "-" : String.format("%.3f", v);
    }
}
