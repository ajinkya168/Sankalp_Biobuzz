package org.firstinspires.ftc.teamcode.test;

import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ivy.Bindings;
import org.firstinspires.ftc.teamcode.robot.RobotConstants.FlowerLevel;
import org.firstinspires.ftc.teamcode.robot.RobotHardware;

/**
 * Bench test for the Flower mechanism: drive the lift to preset levels, jog it manually,
 * and test the wrist/claw independently. Use this to tune the {@code FlowerLevel} tick counts
 * and servo positions in {@link org.firstinspires.ftc.teamcode.robot.RobotConstants}.
 * Every button is registered once in {@link #init()} on a single {@link Bindings} instance - no
 * manual {@code boolean prevX} edge-detection flags, and no framework beyond Ivy.
 */
@TeleOp(name = "TEST: Flower", group = "Test")
public class FlowerTest extends OpMode {

    private final Bindings bindings = new Bindings();

    private final RobotHardware robot = new RobotHardware();

    @Override
    public void init() {
        Scheduler.reset();
        robot.init(hardwareMap, telemetry);

        bindings.onTrue(() -> gamepad1.a, robot.flower.goTo(FlowerLevel.INTAKE));
        bindings.onTrue(() -> gamepad1.b, robot.flower.goTo(FlowerLevel.CARRY));
        bindings.onTrue(() -> gamepad1.y, robot.flower.goTo(FlowerLevel.SCORE));
        bindings.onTrue(() -> gamepad1.x, robot.flower.deployWrist());

        bindings.onTrue(() -> gamepad1.right_bumper, robot.flower.openClaw());
        bindings.onTrue(() -> gamepad1.left_bumper, robot.flower.closeClaw());
    }

    @Override
    public void loop() {
        // Manual override: right stick y jogs the lift target directly.
        double jog = -gamepad1.right_stick_y;
        if (Math.abs(jog) > 0.05) {
            robot.flower.jogLift(jog);
        }

        bindings.poll();
        Scheduler.execute();

        telemetry.addLine("A/B/Y = lift presets (intake/carry/score), X = deploy wrist");
        telemetry.addLine("Bumpers = claw open/close, right stick Y = manual lift jog");
        telemetry.addData("Lift position", robot.flower.getLiftPosition());
        telemetry.update();
    }
}
