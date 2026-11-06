package org.firstinspires.ftc.teamcode.pedro.visualizer.follower;

import org.firstinspires.ftc.teamcode.pedro.visualizer.paths.PathBuilder;

/**
 * Visualizer-export stand-in for the Pedro 2.x Follower. The exported paths class only uses
 * {@code follower.pathBuilder()}; the real Pedro 3.0 follower is created in BaseAuto.
 */
public class Follower {
    public PathBuilder pathBuilder() {
        return new PathBuilder();
    }
}
