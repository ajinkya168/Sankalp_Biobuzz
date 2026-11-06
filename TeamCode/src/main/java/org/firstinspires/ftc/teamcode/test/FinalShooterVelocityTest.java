package org.firstinspires.ftc.teamcode.test;

import static org.firstinspires.ftc.teamcode.hdware.final_Globals.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hdware.FinalHardware;
import org.firstinspires.ftc.teamcode.subsystems.FinalShooterPIDFSubsystem;

/*
 * Shooter velocity test - OUR velocity PIDF (FinalShooterPIDFSubsystem + VelocityPIDF), setPower.
 * Tune live on FTC Dashboard under "final_Globals":
 *   POLLEN_kP / kI / kD / kF + SHOOTER_POLLEN_VELOCITY   (POLLEN mode, kP / kF = shooter1)
 *   NECTAR_kP / kI / kD / kF + SHOOTER_NECTAR_VELOCITY   (NECTAR mode, kP / kF = shooter1)
 *   POLLEN2_kP / kF, NECTAR2_kP / kF                     (shooter2's own kP / kF, kI / kD shared)
 *   SHOOTER_I_MAX, SHOOTER_VOLTAGE_COMP
 * Each motor has its own loop on its own encoder - graph "target" vs "shooter1 vel" / "shooter2 vel".
 * Gains are POWER units: power = kP*error + kI*integral + kD*d(error)/dt + kF*target
 *   (kF ~ 1 / max ticks/s at full power, e.g. 0.00038 - not the hub's 12.x numbers).
 * Graph "target" vs "velocity".
 *
 *   A                 - shooter on / off
 *   right / left bumper - active mode's target +/- SHOOTER_VELOCITY_STEP
 *   right trigger (hold) - intake + feeder (flap open), to see the velocity recover
 *   dpad up / down    - hood +/- HOOD_STEP (starts at HOOD_POLLEN_POS) - copy "hood pos" into final_Globals
 *   start             - POLLEN <-> NECTAR: adjust servo (ADJUST_*_POS), gains (*_k*), target (SHOOTER_*_VELOCITY)
 *   dpad right / left - adjust servo +/- ADJUST_STEP - copy "adjust pos" into ADJUST_<mode>_POS
 *
 * Tuning: kP = kI = kD = 0, raise kF until the velocity is just under the target, then raise kP
 * until it recovers fast after a shot without oscillating, a little kI only if it settles low.
 *
 * Direction: motor directions are FinalHardware's (SHOOTER1/2_REVERSED). Velocities come from
 * robot.getShooter1Velocity() / getShooter2Velocity(), which apply SHOOTER1/2_ENCODER_REVERSED -
 * our PIDF uses those, so a flipped encoder is fine here. Both must read POSITIVE while the
 * flywheel spins the shooting way; below BACKWARDS_LIMIT while on, the test shuts off.
 */
@TeleOp(name = "FINAL: Shooter Velocity PIDF Test", group = "Final Test")
public class FinalShooterVelocityTest extends LinearOpMode {

    private static final double BACKWARDS_LIMIT = -200; // ticks/s, below this while on = direction / encoder flag wrong

    private final FinalHardware robot = FinalHardware.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap);
        FinalShooterPIDFSubsystem shooter = new FinalShooterPIDFSubsystem(robot);
        robot.shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.flap.setPosition(flapOpen);
        double hoodPos = robot.setHood(HOOD_POLLEN_POS);
        boolean nectar = false; // POLLEN
        double adjustPos = robot.setAdjust(ADJUST_POLLEN_POS);

        telemetry.addLine("Ready - A on/off | RB/LB velocity | RT feed | dpad up/down hood | start pollen/nectar | dpad right/left adjust");
        telemetry.update();
        waitForStart();

        boolean aPrev = false, rbPrev = false, lbPrev = false, upPrev = false, downPrev = false;
        boolean startPrev = false, rightPrev = false, leftPrev = false;
        String warning = null;

        while (opModeIsActive()) {
            // buttons
            if (gamepad1.a && !aPrev) {
                shooter.setOn(!shooter.isOn());
                warning = null;
            }
            if (gamepad1.right_bumper && !rbPrev) {
                if (nectar) SHOOTER_NECTAR_VELOCITY += SHOOTER_VELOCITY_STEP;
                else SHOOTER_POLLEN_VELOCITY += SHOOTER_VELOCITY_STEP;
            }
            if (gamepad1.left_bumper && !lbPrev) {
                if (nectar) SHOOTER_NECTAR_VELOCITY = Math.max(0, SHOOTER_NECTAR_VELOCITY - SHOOTER_VELOCITY_STEP);
                else SHOOTER_POLLEN_VELOCITY = Math.max(0, SHOOTER_POLLEN_VELOCITY - SHOOTER_VELOCITY_STEP);
            }
            if (gamepad1.dpad_up && !upPrev) hoodPos = robot.setHood(hoodPos + HOOD_STEP);
            if (gamepad1.dpad_down && !downPrev) hoodPos = robot.setHood(hoodPos - HOOD_STEP);
            if (gamepad1.start && !startPrev) {
                nectar = !nectar;
                adjustPos = robot.setAdjust(nectar ? ADJUST_NECTAR_POS : ADJUST_POLLEN_POS);
            }
            if (gamepad1.dpad_right && !rightPrev) adjustPos = robot.setAdjust(adjustPos + ADJUST_STEP);
            if (gamepad1.dpad_left && !leftPrev) adjustPos = robot.setAdjust(adjustPos - ADJUST_STEP);
            aPrev = gamepad1.a;
            rbPrev = gamepad1.right_bumper;
            lbPrev = gamepad1.left_bumper;
            upPrev = gamepad1.dpad_up;
            downPrev = gamepad1.dpad_down;
            startPrev = gamepad1.start;
            rightPrev = gamepad1.dpad_right;
            leftPrev = gamepad1.dpad_left;

            // shooter - our PIDF (gains + target of the active mode)
            shooter.setNectar(nectar);
            shooter.setTarget(nectar ? SHOOTER_NECTAR_VELOCITY : SHOOTER_POLLEN_VELOCITY);
            shooter.periodic();

            if (shooter.isOn() && (shooter.v1 < BACKWARDS_LIMIT || shooter.v2 < BACKWARDS_LIMIT)) {
                shooter.stop();
                warning = (shooter.v1 < BACKWARDS_LIMIT ? "shooter1" : "shooter2")
                        + " reads backwards - shut off. Wrong way: flip SHOOTER*_REVERSED, "
                        + "right way: flip SHOOTER*_ENCODER_REVERSED";
            }

            // feed
            boolean feeding = gamepad1.right_trigger > 0.5;
            robot.intake.setPower(feeding ? -INTAKE_POWER : 0);
            robot.feeder.setPower(feeding ? FEEDER_POWER : 0);

            // telemetry
            if (warning != null) telemetry.addLine("!! " + warning);
            telemetry.addData("Mode", nectar ? "NECTAR" : "POLLEN");
            telemetry.addData("Shooter", shooter.isOn() ? (shooter.atSpeed() ? "AT SPEED" : "spinning up") : "OFF");
            telemetry.addData("target", shooter.target);
            telemetry.addData("velocity", shooter.velocity);
            telemetry.addData("error", shooter.target - shooter.velocity);
            telemetry.addData("shooter1 vel", shooter.v1);
            telemetry.addData("shooter2 vel", shooter.v2);
            telemetry.addData("shooter1 error", shooter.target - shooter.v1);
            telemetry.addData("shooter2 error", shooter.target - shooter.v2);
            telemetry.addData("shooter1 power", shooter.power1);
            telemetry.addData("shooter2 power", shooter.power2);
            telemetry.addData("shooter1 P term", shooter.pidf1().pTerm);
            telemetry.addData("shooter1 F term", shooter.pidf1().fTerm);
            telemetry.addData("shooter2 P term", shooter.pidf2().pTerm);
            telemetry.addData("shooter2 F term", shooter.pidf2().fTerm);
            telemetry.addData("I term 1 / 2", "%.4f / %.4f", shooter.pidf1().iTerm, shooter.pidf2().iTerm);
            telemetry.addData("D term 1 / 2", "%.4f / %.4f", shooter.pidf1().dTerm, shooter.pidf2().dTerm);
            telemetry.addData("battery (V)", "%.2f", shooter.voltage);
            telemetry.addData("shooter1 kP / kF", "%.5f / %.6f",
                    nectar ? NECTAR_kP : POLLEN_kP, nectar ? NECTAR_kF : POLLEN_kF);
            telemetry.addData("shooter2 kP / kF", "%.5f / %.6f",
                    nectar ? NECTAR2_kP : POLLEN2_kP, nectar ? NECTAR2_kF : POLLEN2_kF);
            telemetry.addData("kI / kD (shared)", "%.5f / %.5f",
                    nectar ? NECTAR_kI : POLLEN_kI, nectar ? NECTAR_kD : POLLEN_kD);
            telemetry.addData("hood pos", "%.3f", hoodPos);
            telemetry.addData("adjust pos", "%.4f", adjustPos);
            telemetry.addData("direction 1 / 2", "%s / %s", robot.shooter1.getDirection(), robot.shooter2.getDirection());
            telemetry.addData("encoder reversed 1 / 2", "%b / %b", SHOOTER1_ENCODER_REVERSED, SHOOTER2_ENCODER_REVERSED);
            telemetry.update();
        }

        shooter.stop();
        robot.intake.setPower(0);
        robot.feeder.setPower(0);
    }
}
