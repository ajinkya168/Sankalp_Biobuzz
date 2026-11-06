package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.api.Paths;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.RobotHardware;

import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;

/**
 * Test autonomous for the BIOBUZZ season, built as a single scheduled Ivy {@link Command}.
 * <p>
 * Drives to the HIVE while spinning up the SSHooter, fires 3 preloaded elements, then drives
 * to the wall and scores a NECTAR on a FLOWER. Positions below are placeholders - TODO replace
 * them with your field-measured poses before running this on a real field.
 */
@Autonomous(name = "BIOBUZZ Auto Test (Ivy)", group = "Test")
public class AutoTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Scheduler.reset();
        RobotHardware robot = new RobotHardware();
        robot.init(hardwareMap, telemetry);

        Pose startPose = new Pose(0, 0, 0);
        Pose hivePose = new Pose(12, 0, 0);
        Pose flowerPose = new Pose(12, 24, Math.toRadians(90));

        robot.drive.follower.setPose(startPose);

        Path toHive = Paths.line(startPose, hivePose).tangent();
        Path toFlower = Paths.line(hivePose, flowerPose).tangent();

        Command routine = sequential(
                // Drive to the HIVE while the flywheels spin up in parallel.
                parallel(
                        robot.drive.followPath(toHive),
                        robot.shooter.spinUp(RobotConstants.SHOOTER_TARGET_VELOCITY)
                                .until(() -> robot.shooter.atTargetVelocity(RobotConstants.SHOOTER_TARGET_VELOCITY))
                ),
                robot.shooter.fireOne(),
                robot.shooter.fireOne(),
                robot.shooter.fireOne(),
                robot.shooter.stop(),

                // Drive to the wall and score a NECTAR on the FLOWER.
                robot.drive.followPath(toFlower),
                robot.flower.scoreOnFlower(),
                robot.flower.stowAll(),
                robot.drive.holdCurrentPose()
        );

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        routine.schedule();
        while (opModeIsActive() && !isStopRequested()) {
            robot.drive.update();
            Scheduler.execute();

            telemetry.addData("Pose", robot.drive.follower.pose());
            telemetry.addData("Shooter velocity", robot.shooter.getVelocity());
            telemetry.addData("Flower lift", robot.flower.getLiftPosition());
            telemetry.update();
        }
    }
}
