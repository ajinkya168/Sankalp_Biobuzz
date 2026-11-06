package org.firstinspires.ftc.teamcode.test;

import static org.firstinspires.ftc.teamcode.hdware.final_Globals.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.hdware.FinalHardware;

/*
 * Simple shooter tuning - the hub's velocity PIDF (setVelocityPIDFCoefficients + setVelocity).
 * Tune live on FTC Dashboard under "final_Globals":
 *   SHOOTER1/2_HUB_kP, SHOOTER_HUB_kI / kD, SHOOTER1/2_HUB_kF, SHOOTER_HUB_TARGET_VELOCITY
 * Graph "target" vs "velocity".
 *
 *   A                    - shooter on / off
 *   right / left bumper  - target +/- SHOOTER_VELOCITY_STEP
 *   right trigger (hold) - shoot pollen: flap open, intake + feeder on
 *   dpad up              - flip shooter1 MOTOR direction     (shooter off only)
 *   dpad down            - flip shooter2 MOTOR direction     (shooter off only)
 *   dpad left            - flip shooter1 ENCODER direction
 *   dpad right           - flip shooter2 ENCODER direction
 * The buttons change SHOOTER1/2_REVERSED and SHOOTER1/2_ENCODER_REVERSED in final_Globals -
 * copy the values that work into final_Globals.java, they reset when the robot restarts.
 *
 * Setting the directions (motors mounted opposite, so exactly one should be reversed):
 *   1. Motor buttons until both motors push the flywheel the shooting way.
 *   2. Encoder buttons until shooter1 vel and shooter2 vel both read POSITIVE.
 * The motor direction flips the power AND the encoder; the encoder flip only flips our reading.
 * The hub loop needs each encoder to count with its power, so if a motor only reads positive
 * with its encoder flipped, the hub can't hold it (runs away) - check that encoder cable /
 * the motor type in the robot config.
 *
 * Tuning: kP = kI = kD = 0, raise kF until the velocity is just under the target, then raise kP
 * until it gets there fast without oscillating, a little kI only if it settles low.
 */
@TeleOp(name = "FINAL: Shooter Tuning", group = "Final Test")
public class FinalShooterTuning extends LinearOpMode {

    private final FinalHardware robot = FinalHardware.getInstance();

    private double appliedP1 = Double.NaN, appliedP2, appliedI, appliedD, appliedF1, appliedF2;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap);
        for (DcMotorEx m : new DcMotorEx[]{robot.shooter1, robot.shooter2}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // hub velocity loop
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        applyDirections();
        applyCoefficients();

        telemetry.addLine("Ready - A on/off | RB/LB target | RT shoot | dpad up/down motor dir | dpad left/right encoder dir");
        telemetry.update();
        waitForStart();

        boolean shooterOn = false;
        boolean aPrev = false, rbPrev = false, lbPrev = false;
        boolean upPrev = false, downPrev = false, leftPrev = false, rightPrev = false;
        String message = "";

        while (opModeIsActive()) {
            //TODO shooter on / off + target
            if (gamepad1.a && !aPrev) shooterOn = !shooterOn;
            if (gamepad1.right_bumper && !rbPrev) SHOOTER_HUB_TARGET_VELOCITY += SHOOTER_VELOCITY_STEP;
            if (gamepad1.left_bumper && !lbPrev) SHOOTER_HUB_TARGET_VELOCITY = Math.max(0, SHOOTER_HUB_TARGET_VELOCITY - SHOOTER_VELOCITY_STEP);

            //TODO direction buttons - motor flips only while off (flipping a spinning flywheel slams it)
            if (gamepad1.dpad_up && !upPrev) {
                if (shooterOn) message = "turn the shooter off (A) before flipping a motor";
                else { SHOOTER1_REVERSED = !SHOOTER1_REVERSED; message = "shooter1 motor flipped"; }
            }
            if (gamepad1.dpad_down && !downPrev) {
                if (shooterOn) message = "turn the shooter off (A) before flipping a motor";
                else { SHOOTER2_REVERSED = !SHOOTER2_REVERSED; message = "shooter2 motor flipped"; }
            }
            if (gamepad1.dpad_left && !leftPrev) {
                SHOOTER1_ENCODER_REVERSED = !SHOOTER1_ENCODER_REVERSED;
                message = "shooter1 encoder flipped";
            }
            if (gamepad1.dpad_right && !rightPrev) {
                SHOOTER2_ENCODER_REVERSED = !SHOOTER2_ENCODER_REVERSED;
                message = "shooter2 encoder flipped";
            }
            aPrev = gamepad1.a;
            rbPrev = gamepad1.right_bumper;
            lbPrev = gamepad1.left_bumper;
            upPrev = gamepad1.dpad_up;
            downPrev = gamepad1.dpad_down;
            leftPrev = gamepad1.dpad_left;
            rightPrev = gamepad1.dpad_right;

            // buttons or dashboard changes
            if (!shooterOn) applyDirections();
            applyCoefficients();

            //TODO shooter - same target to both, the motor directions handle the opposite mounting
            double target = shooterOn ? SHOOTER_HUB_TARGET_VELOCITY : 0;
            robot.shooter1.setVelocity(target);
            robot.shooter2.setVelocity(target);

            // velocity with the encoder flips (FinalHardware getters)
            double v1 = robot.getShooter1Velocity();
            double v2 = robot.getShooter2Velocity();
            double velocity = (v1 + v2) / 2;
            boolean atSpeed = shooterOn && Math.abs(target - velocity) <= SHOOTER_READY_TOL;

            //TODO shoot pollen (right trigger hold) - flap open, intake + feeder
            boolean shooting = gamepad1.right_trigger > 0.5;
            robot.flap.setPosition(shooting ? flapOpen : flapClose);
            robot.intake.setPower(shooting ? -INTAKE_POWER : 0);
            robot.feeder.setPower(shooting ? FEEDER_POWER : 0);

            //TODO telemetry
            if (SHOOTER1_ENCODER_REVERSED || SHOOTER2_ENCODER_REVERSED) {
                telemetry.addLine("!! encoder flipped - the hub loop can't hold that motor, see the note at the top");
            }
            if (!message.isEmpty()) telemetry.addLine(message);
            telemetry.addData("Shooter", shooterOn ? (atSpeed ? "AT SPEED" : "spinning up") : "OFF");
            telemetry.addData("target", target);
            telemetry.addData("velocity", velocity);
            telemetry.addData("error", target - velocity);
            telemetry.addData("shooter1 vel", v1);
            telemetry.addData("shooter2 vel", v2);
            telemetry.addData("shooter1 motor / encoder reversed", "%b / %b", SHOOTER1_REVERSED, SHOOTER1_ENCODER_REVERSED);
            telemetry.addData("shooter2 motor / encoder reversed", "%b / %b", SHOOTER2_REVERSED, SHOOTER2_ENCODER_REVERSED);
            telemetry.addData("kP1 / kP2 / kI / kD / kF1 / kF2", "%.3f / %.3f / %.3f / %.3f / %.3f / %.3f",
                    SHOOTER1_HUB_kP, SHOOTER2_HUB_kP, SHOOTER_HUB_kI, SHOOTER_HUB_kD, SHOOTER1_HUB_kF, SHOOTER2_HUB_kF);
            telemetry.addData("Shooting (RT)", shooting ? "ON" : "OFF");
            telemetry.update();
        }

        robot.shooter1.setVelocity(0);
        robot.shooter2.setVelocity(0);
        robot.intake.setPower(0);
        robot.feeder.setPower(0);
    }

    /** Motor directions from final_Globals (flips the power and the encoder together). */
    private void applyDirections() {
        robot.shooter1.setDirection(SHOOTER1_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        robot.shooter2.setDirection(SHOOTER2_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
    }

    /** Sends the final_Globals gains (own kP / kF per motor) to both motors, only when they change. */
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
}
