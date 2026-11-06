package org.firstinspires.ftc.teamcode.test;

import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ivy.Bindings;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.RobotHardware;

import java.util.List;

import static com.pedropathing.ivy.commands.Commands.instant;

/**
 * Bench test for the Limelight: switch pipelines and read back raw + converted detections,
 * to tune the camera mount geometry constants in {@link RobotConstants}. The pipeline-switch
 * button is registered once in {@link #init()} on a single {@link Bindings} instance - no
 * manual {@code boolean prevSwitch} edge-detection flag, and no framework beyond Ivy.
 */
@TeleOp(name = "TEST: Limelight", group = "Test")
public class LimelightTest extends OpMode {

    private final Bindings bindings = new Bindings();

    private final RobotHardware robot = new RobotHardware();
    private int pipeline = RobotConstants.LIMELIGHT_POLLEN_PIPELINE;

    @Override
    public void init() {
        Scheduler.reset();
        robot.init(hardwareMap, telemetry);
        robot.limelight.setPipeline(pipeline);
        robot.limelight.start();

        bindings.onTrue(() -> gamepad1.a, instant(() -> {
            pipeline = (pipeline == RobotConstants.LIMELIGHT_POLLEN_PIPELINE)
                    ? RobotConstants.LIMELIGHT_NECTAR_PIPELINE
                    : RobotConstants.LIMELIGHT_POLLEN_PIPELINE;
            robot.limelight.setPipeline(pipeline);
        }));
    }

    @Override
    public void loop() {
        bindings.poll();
        Scheduler.execute();

        LLResult result = robot.limelight.getLatestResult();
        telemetry.addData("Pipeline", pipeline);
        telemetry.addData("Valid result", result != null && result.isValid());

        if (result != null && result.isValid()) {
            List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();
            telemetry.addData("Detections", detections.size());
            for (LLResultTypes.DetectorResult d : detections) {
                telemetry.addData(d.getClassName(), "tx=%.1f ty=%.1f", d.getTargetXDegrees(), d.getTargetYDegrees());
            }

            // Bench test only - assumes the robot is stationary at the field origin facing +x.
            Pose robotPose = new Pose(0, 0, 0);
            for (Pose p : robot.limelight.detectPollen(robotPose)) {
                telemetry.addData("Pollen @", "x=%.1f y=%.1f", p.x(), p.y());
            }
            for (Pose p : robot.limelight.detectNectar(robotPose)) {
                telemetry.addData("Nectar @", "x=%.1f y=%.1f", p.x(), p.y());
            }
        }

        telemetry.addLine("A = switch pipeline (pollen/nectar)");
        telemetry.update();
    }
}
