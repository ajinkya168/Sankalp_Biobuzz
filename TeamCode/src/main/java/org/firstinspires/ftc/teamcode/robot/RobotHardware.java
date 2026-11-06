package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FlowerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SSHooterSubsystem;

/**
 * Single source of truth for every hardware device on the robot.
 * <p>
 * Create one of these as a field, then call {@link #init(HardwareMap, Telemetry)} once from
 * an OpMode's {@code init()} to map every device and use the exposed subsystems - never call
 * {@code hardwareMap.get(...)} directly anywhere else. That keeps device names in one place
 * ({@link RobotConstants}) and lets autonomous, teleop, and every OpMode in the {@code test}
 * package share identical wiring.
 *
 * <pre>{@code
 * private final RobotHardware robot = new RobotHardware();
 *
 * public void init() {
 *     robot.init(hardwareMap, telemetry);
 *     robot.intake.in().schedule();
 *     robot.shooter.shootOnce(1500).schedule();
 * }
 * }</pre>
 */
public class RobotHardware {

    public HardwareMap hardwareMap;
    public Telemetry telemetry;

    public Follower follower;

    public DriveSubsystem drive;
    public IntakeSubsystem intake;
    public SSHooterSubsystem shooter;
    public FlowerSubsystem flower;
    public LimelightSubsystem limelight;

    /** Maps every hardware device and constructs every subsystem. Call once, from an OpMode's init(). */
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        follower = Constants.create(hardwareMap);

        drive = new DriveSubsystem(follower);
        intake = new IntakeSubsystem(hardwareMap);
        shooter = new SSHooterSubsystem(hardwareMap);
        flower = new FlowerSubsystem(hardwareMap);
        limelight = new LimelightSubsystem(hardwareMap);
    }
}
