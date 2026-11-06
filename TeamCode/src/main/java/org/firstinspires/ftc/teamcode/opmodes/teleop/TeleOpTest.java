package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ivy.Bindings;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.RobotHardware;

/**
 * Test teleop for the BIOBUZZ season, built entirely on the Ivy command scheduler - no other
 * command framework or scheduler is involved.
 * <p>
 * Driving is direct/continuous (gamepad1 sticks -> Follower every loop); every discrete
 * mechanism action is registered once in {@link #init()} on a single {@link Bindings} instance
 * (created once, as a field, and reused for every button below) - no hand-rolled
 * {@code boolean prevX} edge-detection flags.
 * <p>
 * Controls:
 * <ul>
 *   <li>gamepad1 sticks - robot-centric driving</li>
 *   <li>gamepad1 right bumper / left bumper - intake in / out (held)</li>
 *   <li>gamepad2 right bumper - toggle SSHooter spin-up</li>
 *   <li>gamepad2 a - fire one element into the HIVE</li>
 *   <li>gamepad2 dpad up - score the held NECTAR on the FLOWER</li>
 *   <li>gamepad2 dpad down - stow the Flower mechanism</li>
 * </ul>
 */
@TeleOp(name = "BIOBUZZ TeleOp Test (Ivy)", group = "Test")
public class TeleOpTest extends OpMode {

    private final Bindings bindings = new Bindings();

    private final RobotHardware robot = new RobotHardware();

    @Override
    public void init() {
        Scheduler.reset();
        robot.init(hardwareMap, telemetry);

        // Intake: run while held, stop the instant the bumper is released.
        bindings.whileTrue(() -> gamepad1.right_bumper, robot.intake.in());
        bindings.whileTrue(() -> gamepad1.left_bumper, robot.intake.out());

        // SSHooter: one press spins it up, the next press spins it back down.
        bindings.toggleOnTrue(() -> gamepad2.right_bumper,
                robot.shooter.spinUpHeld(RobotConstants.SHOOTER_TARGET_VELOCITY));
        bindings.onTrue(() -> gamepad2.a, robot.shooter.fireOne());

        // Flower: one-shot score / stow sequences.
        bindings.onTrue(() -> gamepad2.dpad_up, robot.flower.scoreOnFlower());
        bindings.onTrue(() -> gamepad2.dpad_down, robot.flower.stowAll());
    }

    @Override
    public void loop() {
        // Drivetrain: direct, continuous control every loop.
        robot.drive.driveManual(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x
        );
        robot.drive.update();

        bindings.poll();
        Scheduler.execute();

        telemetry.addData("Shooter velocity", robot.shooter.getVelocity());
        telemetry.addData("Flower lift", robot.flower.getLiftPosition());
        telemetry.update();
    }
}
