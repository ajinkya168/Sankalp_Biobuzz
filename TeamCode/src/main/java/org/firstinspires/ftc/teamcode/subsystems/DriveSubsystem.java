package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.pedro.PedroCommands.hold;

/** Thin wrapper around the Pedro Pathing {@link Follower}, exposing it as Ivy commands. */
public class DriveSubsystem {

    public final Follower follower;

    public DriveSubsystem(Follower follower) {
        this.follower = follower;
    }

    /** Robot-centric manual driving from gamepad sticks. Call every teleop loop, then {@link #update()}. */
    public void driveManual(double forward, double lateral, double turn) {
        follower.manual(forward, lateral, turn);
    }

    /** Must be called exactly once per loop, in both autonomous and teleop. */
    public void update() {
        follower.update();
    }

    /** Follows {@code path}, finishing once the follower reaches the end of it. */
    public Command followPath(Path path) {
        return follow(follower, path).requiring(follower);
    }

    /** Holds the follower's current pose. */
    public Command holdCurrentPose() {
        return hold(follower).requiring(follower);
    }

    /** Holds a specific field pose. */
    public Command holdPose(Pose pose) {
        return hold(follower, pose).requiring(follower);
    }
}
