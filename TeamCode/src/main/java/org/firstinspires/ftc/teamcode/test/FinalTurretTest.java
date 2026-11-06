package org.firstinspires.ftc.teamcode.test;

import static org.firstinspires.ftc.teamcode.hdware.final_Globals.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hdware.FinalHardware;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/*
 * Final Turret Test (same as the sankalp27 Turret Test). Two servos mounted opposite
 * (turret1 + mirrored turret2), 1:1 to the turret. Heading from the Pedro follower (navX).
 * Values in final_Globals (TURRET_*).
 *
 *   A                  - toggle TRACK / INIT
 *                          TRACK: turret holds the FIELD angle TURRET_TARGET_DEG while the robot turns
 *                          INIT : turret goes to TURRET_INIT_POS (facing robot-forward)
 *   dpad left / right  - target field angle +/- TURRET_TARGET_STEP_DEG (counter-clockwise +)
 *   right stick button - zero the heading (robot's current facing = field 0)
 *   sticks             - robot-centric drive, so you can turn the robot and watch the turret hold
 *
 * First checks:
 *   1. INIT: turret must face robot-forward. If not, tune TURRET_INIT_POS.
 *   2. TRACK with target +90: turret must turn LEFT (counter-clockwise). If it turns right,
 *      set TURRET_REVERSED = true.
 *   3. Target +90 should be exactly 90 deg. If it over/undershoots, tune TURRET_SERVO_RANGE_DEG.
 *   4. Both servos must agree (no fighting / buzzing). If they don't, tune TURRET2_TRIM.
 */
@TeleOp(name = "FINAL: Turret Test", group = "Final Test")
public class FinalTurretTest extends LinearOpMode {

    private final FinalHardware robot = FinalHardware.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap);
        Follower follower = Constants.create(hardwareMap);

        double sentPos = robot.setTurretPosition(TURRET_INIT_POS);

        while (opModeInInit()) {
            follower.update();
            telemetry.addLine("Final Turret Test - A = track / init, dpad L/R = target, R-stick press = zero heading");
            telemetry.addLine("Keep the robot still while the navX calibrates");
            telemetry.update();
        }
        if (isStopRequested()) return;

        boolean tracking = false;
        boolean aPrev = false, leftPrev = false, rightPrev = false, rsPrev = false;
        double headingOffset = 0.0;
        ElapsedTime loopTimer = new ElapsedTime();

        while (opModeIsActive()) {
            double loopMs = loopTimer.milliseconds();
            loopTimer.reset();

            //TODO buttons
            if (gamepad1.a && !aPrev) tracking = !tracking;
            aPrev = gamepad1.a;
            if (gamepad1.dpad_left && !leftPrev) TURRET_TARGET_DEG = AngleUnit.normalizeDegrees(TURRET_TARGET_DEG + TURRET_TARGET_STEP_DEG);
            if (gamepad1.dpad_right && !rightPrev) TURRET_TARGET_DEG = AngleUnit.normalizeDegrees(TURRET_TARGET_DEG - TURRET_TARGET_STEP_DEG);
            leftPrev = gamepad1.dpad_left;
            rightPrev = gamepad1.dpad_right;
            if (gamepad1.right_stick_button && !rsPrev) headingOffset = follower.pose().heading();
            rsPrev = gamepad1.right_stick_button;

            //TODO turret
            double headingDeg = AngleUnit.normalizeDegrees(Math.toDegrees(follower.pose().heading() - headingOffset));
            double wantedRobotDeg = AngleUnit.normalizeDegrees(TURRET_TARGET_DEG - headingDeg);
            double wantedPos = tracking ? FinalHardware.turretAngleToPosition(wantedRobotDeg) : TURRET_INIT_POS;
            sentPos = robot.setTurretPosition(wantedPos);
            boolean atLimit = tracking && Math.abs(wantedPos - sentPos) > 1e-6;

            //TODO drive (robot-centric, to turn the robot) - Pedro: forward +, strafe left +, turn counter-clockwise +
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * StrafeSpeed;
            double rx = gamepad1.right_stick_x * turnSpeed;
            follower.manual(y, -x, -rx);
            follower.update();

            //TODO telemetry
            telemetry.addData("Mode", tracking ? "TRACK (field angle)" : "INIT position");
            telemetry.addData("Target field angle (deg)", "%.1f", TURRET_TARGET_DEG);
            telemetry.addData("Robot heading (deg)", "%.1f", headingDeg);
            telemetry.addData("Turret wanted, robot-rel (deg)", "%.1f", tracking ? wantedRobotDeg : 0.0);
            telemetry.addData("Turret actual, robot-rel (deg)", "%.1f", FinalHardware.turretPositionToAngle(sentPos));
            telemetry.addData("Turret field angle (deg)", "%.1f",
                    AngleUnit.normalizeDegrees(FinalHardware.turretPositionToAngle(sentPos) + headingDeg));
            telemetry.addData("Wanted pos / sent pos", "%.4f / %.4f%s", wantedPos, sentPos, atLimit ? "  AT LIMIT" : "");
            telemetry.addData("turret1 / turret2 pos", "%.4f / %.4f", robot.turret1.getPosition(), robot.turret2.getPosition());
            telemetry.addData("Loop time (ms)", "%.1f", loopMs);
            telemetry.addLine("A = track/init | dpad L/R = target +/- | R-stick press = zero heading | sticks = drive");
            telemetry.update();
        }

        follower.stop();
    }
}
