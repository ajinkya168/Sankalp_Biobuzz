package org.firstinspires.ftc.teamcode.test;

import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.RobotHardware;

/**
 * Bench test for the SSHooter: spin the flywheels, adjust the target velocity, and fire the
 * feeder, to find {@link RobotConstants#SHOOTER_TARGET_VELOCITY} and the feeder servo positions.
 */
@TeleOp(name = "TEST: Shooter", group = "Test")
public class ShooterTest extends OpMode {

    private final RobotHardware robot = new RobotHardware();
    private double targetVelocity = RobotConstants.SHOOTER_TARGET_VELOCITY;

    private boolean prevFeed = false;
    private boolean prevUp = false;
    private boolean prevDown = false;

    @Override
    public void init() {
        Scheduler.reset();
        robot.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up && !prevUp) targetVelocity += 100;
        if (gamepad1.dpad_down && !prevDown) targetVelocity -= 100;
        prevUp = gamepad1.dpad_up;
        prevDown = gamepad1.dpad_down;

        if (gamepad1.a) {
            robot.shooter.setVelocity(targetVelocity);
        } else {
            robot.shooter.setVelocity(0);
        }

        if (gamepad1.right_bumper && !prevFeed) {
            robot.shooter.fireOne().schedule();
        }
        prevFeed = gamepad1.right_bumper;

        Scheduler.execute();

        telemetry.addLine("A = spin flywheels, RB = feed one, Dpad up/down = adjust target");
        telemetry.addData("Target velocity", targetVelocity);
        telemetry.addData("Actual velocity", robot.shooter.getVelocity());
        telemetry.addData("At target", robot.shooter.atTargetVelocity(targetVelocity));
        telemetry.update();
    }
}
