package org.firstinspires.ftc.teamcode.opmodes.auto.paths;

// Visualizer export, with the 2 edits from opmodes/auto/README.md:
//   1. package line -> org.firstinspires.ftc.teamcode.opmodes.auto.paths
//   2. "com.pedropathing." in the imports -> "org.firstinspires.ftc.teamcode.pedro.visualizer."
import org.firstinspires.ftc.teamcode.pedro.visualizer.follower.Follower;
import org.firstinspires.ftc.teamcode.pedro.visualizer.geometry.BezierCurve;
import org.firstinspires.ftc.teamcode.pedro.visualizer.geometry.BezierLine;
import org.firstinspires.ftc.teamcode.pedro.visualizer.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedro.visualizer.paths.PathChain;

/**
 * PLACEHOLDER paths, in the same shape the Pedro visualizer exports. Replace with your own.
 * Order: score preload -> collect -> score -> park.
 */
public class ExamplePaths {

    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(56.000, 8.000), new Pose(60.000, 84.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(new BezierCurve(new Pose(60.000, 84.000), new Pose(55.000, 84.000), new Pose(20.000, 84.000)))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(20.000, 84.000), new Pose(60.000, 84.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(60.000, 84.000), new Pose(38.000, 60.000)))
                    .setTangentHeadingInterpolation()
                    .build();
        }
    }
}
