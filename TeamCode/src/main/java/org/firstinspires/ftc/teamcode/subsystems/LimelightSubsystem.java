package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.RobotConstants;

import java.util.ArrayList;
import java.util.List;

/**
 * Wraps the Limelight 3A neural detector and converts its detections into field-frame
 * {@link Pose}s, using the camera mount geometry in {@link RobotConstants}.
 */
public class LimelightSubsystem {

    private final Limelight3A limelight;

    public LimelightSubsystem(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, RobotConstants.LIMELIGHT_NAME);
    }

    public void setPipeline(int index) {
        limelight.pipelineSwitch(index);
    }

    public void start() {
        limelight.start();
    }

    public void stop() {
        limelight.stop();
    }

    public LLResult getLatestResult() {
        return limelight.getLatestResult();
    }

    /**
     * Converts every detection matching {@code className} in the latest snapshot into a
     * field-frame Pose. {@code targetHeightIn} is the game element's center height off the field.
     */
    public List<Pose> detect(Pose robotPose, String className, double targetHeightIn) {
        List<Pose> poses = new ArrayList<>();
        LLResult result = getLatestResult();
        if (result == null || !result.isValid()) {
            return poses;
        }

        double cosH = Math.cos(robotPose.heading());
        double sinH = Math.sin(robotPose.heading());
        double mountAngleRad = Math.toRadians(RobotConstants.CAMERA_MOUNT_ANGLE_DEG);
        double heightDeltaIn = RobotConstants.CAMERA_HEIGHT_IN - targetHeightIn;

        for (LLResultTypes.DetectorResult detection : result.getDetectorResults()) {
            if (!className.equalsIgnoreCase(detection.getClassName())) continue;

            double txRad = Math.toRadians(detection.getTargetXDegrees());
            double tyRad = Math.toRadians(detection.getTargetYDegrees());

            // Distance from camera lens to the game element, from the vertical angle and known heights.
            double totalVerticalAngle = mountAngleRad + tyRad;
            double forwardFromCamera = heightDeltaIn / Math.tan(totalVerticalAngle);
            double lateralFromCamera = forwardFromCamera * Math.tan(txRad);

            // Camera frame -> robot frame.
            double forwardFromRobot = forwardFromCamera + RobotConstants.CAMERA_FORWARD_OFFSET_IN;
            double lateralFromRobot = lateralFromCamera + RobotConstants.CAMERA_LATERAL_OFFSET_IN;

            // Robot frame -> field frame using the robot's current heading.
            double fieldX = robotPose.x() + forwardFromRobot * cosH - lateralFromRobot * sinH;
            double fieldY = robotPose.y() + forwardFromRobot * sinH + lateralFromRobot * cosH;

            poses.add(new Pose(fieldX, fieldY, 0));
        }

        return poses;
    }

    public List<Pose> detectPollen(Pose robotPose) {
        return detect(robotPose, "pollen", RobotConstants.POLLEN_HEIGHT_IN);
    }

    public List<Pose> detectNectar(Pose robotPose) {
        return detect(robotPose, "nectar", RobotConstants.NECTAR_HEIGHT_IN);
    }
}
