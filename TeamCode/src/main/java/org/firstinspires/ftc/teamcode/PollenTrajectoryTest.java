package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.ArrayList;
import java.util.List;

/**
 * Drive-only autonomous test.
 * <p>
 * 1) Drives 10" straight forward from (0,0,0) to (10,0,0).
 * 2) Takes a single Limelight neural-detector snapshot and converts each
 *    detected "pollen" centroid into a field-frame Pose using the camera's
 *    mount geometry.
 * 3) Orders the detected pollen nearest-first from the robot's current pose
 *    and follows one continuous Pedro Pathing trajectory through all of them.
 * <p>
 * No intake or other subsystem is touched - this only exercises the
 * drivetrain/Follower and the vision-to-trajectory pipeline.
 */
@Autonomous(name = "Pollen Trajectory Test (Drive Only)")
public class PollenTrajectoryTest extends LinearOpMode {

    // ---- Limelight pipeline ----
    private static final int POLLEN_PIPELINE_INDEX = 0;
    private static final String POLLEN_CLASS_NAME = "pollen"; // must match your trained model's class label

    // ---- Camera mount geometry, inches/degrees - TODO: measure on your robot ----
    private static final double CAMERA_HEIGHT_IN = 9.5;
    private static final double CAMERA_MOUNT_ANGLE_DEG = 20.0; // upward tilt from horizontal
    private static final double POLLEN_HEIGHT_IN = 2.0;        // pollen center height off the field
    private static final double CAMERA_FORWARD_OFFSET_IN = 6.0; // lens forward of robot center
    private static final double CAMERA_LATERAL_OFFSET_IN = 0.0; // lens left(+)/right(-) of robot center

    private static final double FORWARD_DRIVE_IN = 10.0;
    private static final int EXPECTED_POLLEN_COUNT = 3;

    private Follower follower;
    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        follower = Constants.create(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(POLLEN_PIPELINE_INDEX);

        Pose startPose = new Pose(0, 0, 0);
        follower.setPose(startPose);

        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        limelight.start();

        driveForward(startPose, FORWARD_DRIVE_IN);

        List<Pose> pollenPoses = detectPollen(follower.pose());
        telemetry.addData("Pollen found", pollenPoses.size());
        if (pollenPoses.size() != EXPECTED_POLLEN_COUNT) {
            telemetry.addData("Warning", "Expected %d pollen, found %d", EXPECTED_POLLEN_COUNT, pollenPoses.size());
        }
        telemetry.update();

        if (!pollenPoses.isEmpty()) {
            followPollenTrajectory(follower.pose(), pollenPoses);
        }

        limelight.stop();
        follower.stop();

        telemetry.addLine("Done");
        telemetry.update();
    }

    /** Phase 1: open-loop leg, independent of vision. */
    private void driveForward(Pose startPose, double distanceIn) {
        Pose target = startPose.withX(startPose.x() + distanceIn);
        Path forwardLeg = Paths.line(startPose, target).constant(startPose.heading());
        runPath(forwardLeg);
    }

    /** Phase 2: single Limelight snapshot -> field-frame pollen positions. */
    private List<Pose> detectPollen(Pose robotPose) {
        List<Pose> poses = new ArrayList<>();
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return poses;
        }

        double cosH = Math.cos(robotPose.heading());
        double sinH = Math.sin(robotPose.heading());
        double mountAngleRad = Math.toRadians(CAMERA_MOUNT_ANGLE_DEG);
        double heightDeltaIn = CAMERA_HEIGHT_IN - POLLEN_HEIGHT_IN;

        for (LLResultTypes.DetectorResult detection : result.getDetectorResults()) {
            if (!POLLEN_CLASS_NAME.equalsIgnoreCase(detection.getClassName())) continue;

            double txRad = Math.toRadians(detection.getTargetXDegrees());
            double tyRad = Math.toRadians(detection.getTargetYDegrees());

            // Distance from camera lens to pollen, from the vertical angle and known heights.
            double totalVerticalAngle = mountAngleRad + tyRad;
            double forwardFromCamera = heightDeltaIn / Math.tan(totalVerticalAngle);
            double lateralFromCamera = forwardFromCamera * Math.tan(txRad);

            // Camera frame -> robot frame.
            double forwardFromRobot = forwardFromCamera + CAMERA_FORWARD_OFFSET_IN;
            double lateralFromRobot = lateralFromCamera + CAMERA_LATERAL_OFFSET_IN;

            // Robot frame -> field frame using the robot's current heading.
            double fieldX = robotPose.x() + forwardFromRobot * cosH - lateralFromRobot * sinH;
            double fieldY = robotPose.y() + forwardFromRobot * sinH + lateralFromRobot * cosH;

            poses.add(new Pose(fieldX, fieldY, 0));
        }

        return poses;
    }

    /** Phase 3: nearest-first ordering, then one continuous curve through all waypoints. */
    private void followPollenTrajectory(Pose robotPose, List<Pose> detected) {
        List<Pose> ordered = new ArrayList<>();
        List<Pose> remaining = new ArrayList<>(detected);
        Pose cursor = robotPose;

        while (!remaining.isEmpty()) {
            Pose nearest = remaining.get(0);
            double nearestDist = cursor.distance(nearest);
            for (Pose candidate : remaining) {
                double dist = cursor.distance(candidate);
                if (dist < nearestDist) {
                    nearest = candidate;
                    nearestDist = dist;
                }
            }
            ordered.add(nearest);
            remaining.remove(nearest);
            cursor = nearest;
        }

        Pose[] wayPoints = new Pose[ordered.size() + 1];
        wayPoints[0] = robotPose;
        for (int i = 0; i < ordered.size(); i++) {
            wayPoints[i + 1] = ordered.get(i);
        }

        // Path1.through(...) builds one smooth curve interpolating all waypoints in order,
        // so the robot never fully stops between pollen.
        Path pollenTrajectory = Paths.through(wayPoints).tangent();
        runPath(pollenTrajectory);
    }

    private void runPath(Path path) {
        follower.follow(path);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            Pose pose = follower.pose();
            telemetry.addData("x", pose.x());
            telemetry.addData("y", pose.y());
            telemetry.addData("heading (deg)", Math.toDegrees(pose.heading()));
            telemetry.update();
        }
    }
}
