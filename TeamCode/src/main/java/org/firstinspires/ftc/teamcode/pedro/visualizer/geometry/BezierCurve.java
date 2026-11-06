package org.firstinspires.ftc.teamcode.pedro.visualizer.geometry;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/** Visualizer-export stand-in for the Pedro 2.x BezierCurve: start point, control point(s), end point. */
public class BezierCurve {
    private final List<Pose> points;

    public BezierCurve(Pose... points) {
        this(Arrays.asList(points));
    }

    public BezierCurve(List<Pose> points) {
        if (points.size() < 2) throw new IllegalArgumentException("A curve needs at least 2 points");
        this.points = Collections.unmodifiableList(points);
    }

    public List<Pose> getControlPoints() {
        return points;
    }

    public Pose first() {
        return points.get(0);
    }

    public Pose last() {
        return points.get(points.size() - 1);
    }
}
