package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.RobotHardware;

/**
 * Bench test for the intake: run it in/out at variable power with the triggers.
 * Uses {@link RobotHardware} for hardware mapping, so device names always match production code.
 */
@TeleOp(name = "TEST: Intake", group = "Test")
public class IntakeTest extends OpMode {

    private final RobotHardware robot = new RobotHardware();

    @Override
    public void init() {
        robot.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        if (gamepad1.right_trigger > 0.05) {
            robot.intake.setPower(gamepad1.right_trigger);
        } else if (gamepad1.left_trigger > 0.05) {
            robot.intake.setPower(-gamepad1.left_trigger);
        } else {
            robot.intake.setPower(0);
        }

        telemetry.addLine("Right trigger = intake in, Left trigger = intake out");
        telemetry.addData("Commanded power", robot.intake.getPower());
        telemetry.update();
    }
}
