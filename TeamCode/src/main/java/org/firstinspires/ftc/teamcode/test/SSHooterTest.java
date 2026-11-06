package org.firstinspires.ftc.teamcode.test;

import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ivy.Bindings;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.RobotHardware;

import static com.pedropathing.ivy.commands.Commands.instant;

/**
 * Bench test for the SSHooter: spin the flywheels, adjust the target velocity, and fire the
 * feeder, to find {@link RobotConstants#SHOOTER_TARGET_VELOCITY} and the feeder servo positions.
 * Every button is registered once in {@link #init()} on a single {@link Bindings} instance - no
 * manual {@code boolean prevX} edge-detection flags, and no framework beyond Ivy.
 */
@TeleOp(name = "TEST: SSHooter", group = "Test")
public class SSHooterTest extends OpMode {

    private final Bindings bindings = new Bindings();

    private final RobotHardware robot = new RobotHardware();
    private double targetVelocity = RobotConstants.SHOOTER_TARGET_VELOCITY;

    @Override
    public void init() {
        Scheduler.reset();
        robot.init(hardwareMap, telemetry);

        bindings.onTrue(() -> gamepad1.dpad_up, instant(() -> targetVelocity += 100));
        bindings.onTrue(() -> gamepad1.dpad_down, instant(() -> targetVelocity -= 100));

        // Held: spins at the *current* targetVelocity, live, even as the dpad adjusts it above.
        bindings.whileTrue(() -> gamepad1.a, robot.shooter.spinUpHeld(() -> targetVelocity));
        bindings.onTrue(() -> gamepad1.right_bumper, robot.shooter.fireOne());
    }

    @Override
    public void loop() {
        bindings.poll();
        Scheduler.execute();

        telemetry.addLine("A = spin flywheels, RB = feed one, Dpad up/down = adjust target");
        telemetry.addData("Target velocity", targetVelocity);
        telemetry.addData("Actual velocity", robot.shooter.getVelocity());
        telemetry.addData("At target", robot.shooter.atTargetVelocity(targetVelocity));
        telemetry.update();
    }
}
