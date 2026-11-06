package org.firstinspires.ftc.teamcode.robot;

/**
 * Driver Station configuration names and tunable values for every subsystem.
 * <p>
 * The device names below must match your robot's configuration file exactly.
 * The numeric values are starting points only - retune them with the matching
 * OpMode in the {@code test} package (SSHooterTest, IntakeTest, FlowerTest,
 * LimelightTest) before relying on them in autonomous or teleop.
 */
public final class RobotConstants {

    private RobotConstants() {
    }

    // ===================== Driver Station configuration names =====================
    // Drivetrain motors/localizer are wired up through Pedro Pathing - see
    // org.firstinspires.ftc.teamcode.pedro.Constants, not this file.

    public static final String INTAKE_MOTOR_NAME = "intake";

    public static final String SHOOTER_LEFT_MOTOR_NAME = "shooterLeft";
    public static final String SHOOTER_RIGHT_MOTOR_NAME = "shooterRight";
    public static final String SHOOTER_FEEDER_SERVO_NAME = "shooterFeeder";

    public static final String FLOWER_LIFT_MOTOR_NAME = "flowerLift";
    public static final String FLOWER_WRIST_SERVO_NAME = "flowerWrist";
    public static final String FLOWER_CLAW_SERVO_NAME = "flowerClaw";

    public static final String LIMELIGHT_NAME = "limelight";

    // ===================== Intake =====================
    // TODO tune with IntakeTest
    public static final double INTAKE_IN_POWER = 1.0;
    public static final double INTAKE_OUT_POWER = -1.0;

    // ===================== SSHooter (launches POLLEN/NECTAR into the HIVE) =====================
    // TODO tune with SSHooterTest
    public static final double SHOOTER_TARGET_VELOCITY = 1500; // motor ticks/sec
    public static final double SHOOTER_VELOCITY_TOLERANCE = 50; // motor ticks/sec
    public static final double SHOOTER_REST_POSITION = 0.20;
    public static final double SHOOTER_FEED_POSITION = 0.65;
    public static final long SHOOTER_FEED_TIME_MS = 200;

    // ===================== Flower (carries a NECTAR up to score on a wall FLOWER) =====================
    // TODO tune with FlowerTest
    public static final double FLOWER_LIFT_POWER = 0.85;
    public static final int FLOWER_LIFT_TOLERANCE_TICKS = 15;
    public static final int FLOWER_JOG_TICKS_PER_LOOP = 15;

    public static final double FLOWER_WRIST_STOW_POSITION = 0.05;
    public static final double FLOWER_WRIST_SCORE_POSITION = 0.75;
    public static final double FLOWER_CLAW_OPEN_POSITION = 0.40;
    public static final double FLOWER_CLAW_CLOSED_POSITION = 0.00;

    /** Preset lift heights, in encoder ticks from the zeroed (fully retracted) position. */
    public enum FlowerLevel {
        INTAKE(0),
        CARRY(300),
        SCORE(1450);

        public final int ticks;

        FlowerLevel(int ticks) {
            this.ticks = ticks;
        }
    }

    // ===================== Limelight vision =====================
    // TODO tune with LimelightTest - these describe the camera's mount on the robot.
    public static final int LIMELIGHT_POLLEN_PIPELINE = 0;
    public static final int LIMELIGHT_NECTAR_PIPELINE = 1;

    public static final double CAMERA_HEIGHT_IN = 9.5;
    public static final double CAMERA_MOUNT_ANGLE_DEG = 20.0; // upward tilt from horizontal
    public static final double CAMERA_FORWARD_OFFSET_IN = 6.0; // lens forward of robot center
    public static final double CAMERA_LATERAL_OFFSET_IN = 0.0; // lens left(+)/right(-) of robot center

    public static final double POLLEN_HEIGHT_IN = 2.0; // POLLEN ball center height off the field
    public static final double NECTAR_HEIGHT_IN = 2.5; // NECTAR ball center height off the field
}
