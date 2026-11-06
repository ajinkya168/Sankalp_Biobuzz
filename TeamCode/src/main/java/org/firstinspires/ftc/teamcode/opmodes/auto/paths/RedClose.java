package org.firstinspires.ftc.teamcode.opmodes.auto.paths;

import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "RedClose", group = "Autonomous")
public class RedClose extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56.3594, 8.8984, 180);
    private final Pose start_2 = poseFactory.of(56.1467, 9.4148, 180);
    private final Pose gardenIntake = poseFactory.of(13.9543, 9.407, 180);
    private final Pose flowerIntakeStart = poseFactory.of(13.9543, 9.407, 180);
    private final Pose flowerIntake = poseFactory.of(47.684, 126.6051, 90);
    private final Pose flowerIntakeControl1 = poseFactory.of(19.6785, 89.1304, 0);
    private final Pose parkStart = poseFactory.of(47.684, 126.6051, 90);
    private final Pose park = poseFactory.of(9.3408, 109.4643, 90);

    // Pause (seconds) after each path. The visualizer export does NOT include waits - set them here.
    public static double WAIT_AFTER_GARDEN_INTAKE_S = 1.0;
    public static double WAIT_AFTER_FLOWER_INTAKE_S = 2.0;

    // Max time to wait for the robot to stop at the end of a path before moving on.
    public static double SETTLE_TIMEOUT_S = 1.0;

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                // startPath() removed: it is only 0.56 in long (visualizer leftover) and just makes the robot twitch.
                drive(startPath()),
                waitMs(500),
                drive(gardenIntake()),
                waitMs(WAIT_AFTER_GARDEN_INTAKE_S * 1000),
                drive(flowerIntake()),
                waitMs(WAIT_AFTER_FLOWER_INTAKE_S * 1000),
                drive(park())
        );
    }

    /**
     * Ivy's follow() ends as soon as the robot passes the END of the curve (atParametricEnd), while it
     * is still moving fast, so the next path starts mid-motion and the robot wobbles. Here we also wait
     * until Pedro says it has settled at the end point (it holds the end pose meanwhile), with a timeout.
     */
    private Command drive(Path path) {
        return sequential(
                follow(follower, path),
                waitUntil(() -> !follower.isBusy()).raceWith(waitMs(SETTLE_TIMEOUT_S * 1000))
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    public Path startPath() {
        return Paths.line(start, start_2).constant(start_2);
    }

    public Path gardenIntake() {
        return Paths.line(start, gardenIntake).constant(gardenIntake);
    }

    public Path flowerIntake() {
        return Paths.curve(flowerIntakeStart, flowerIntakeControl1, flowerIntake).linear(flowerIntakeStart, flowerIntake);
    }

    public Path park() {
        return Paths.line(parkStart, park).linear(parkStart, park);
    }
}
