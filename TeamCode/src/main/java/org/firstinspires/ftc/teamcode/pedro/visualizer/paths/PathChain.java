package org.firstinspires.ftc.teamcode.pedro.visualizer.paths;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.pedro.visualizer.geometry.BezierCurve;
import org.firstinspires.ftc.teamcode.pedro.visualizer.geometry.Pose;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Visualizer-export stand-in for the Pedro 2.x PathChain: one or more curves, each with its heading.
 * {@link #toPath()} turns it into the Pedro 3.0 {@link Path} the follower runs.
 */
public class PathChain {

    /** One curve of the chain plus its heading interpolation. */
    public static final class Segment {
        public final BezierCurve curve;
        public final HeadingInterpolator heading;

        Segment(BezierCurve curve, HeadingInterpolator heading) {
            this.curve = curve;
            this.heading = heading;
        }
    }

    private final List<Segment> segments;

    PathChain(List<Segment> segments) {
        if (segments.isEmpty()) throw new IllegalStateException("PathChain has no paths - addPath(...) before build()");
        this.segments = Collections.unmodifiableList(new ArrayList<>(segments));
    }

    public List<Segment> segments() {
        return segments;
    }

    /** Pose at the very start of the chain, heading included (where the robot must be placed). */
    public Pose startPose() {
        Segment s = segments.get(0);
        return s.curve.first().withHeading(headingAtStart(s));
    }

    /** Pose at the very end of the chain. */
    public Pose endPose() {
        Segment s = segments.get(segments.size() - 1);
        return s.curve.last().withHeading(headingAtEnd(s));
    }

    /** Pedro 3.0 path for this chain (a compound path when the chain has more than one curve). */
    public Path toPath() {
        if (segments.size() == 1) return convert(segments.get(0));
        Path[] parts = new Path[segments.size()];
        for (int i = 0; i < parts.length; i++) parts[i] = convert(segments.get(i));
        return Paths.path(parts);
    }

    private static Path convert(Segment s) {
        List<Pose> pts = s.curve.getControlPoints();
        Path path;
        if (pts.size() == 2 && !LINE_COMPLETION_BROKEN) {
            path = Paths.line(pts.get(0).toVector(), pts.get(1).toVector());
        } else if (pts.size() == 2) {     // Bezier with the midpoint as control = the same straight line
            Vector2D a = pts.get(0).toVector(), b = pts.get(1).toVector();
            path = Paths.curve(a, a.plus(b).div(2), b);
        } else {
            Vector2D[] v = new Vector2D[pts.size()];
            for (int i = 0; i < v.length; i++) v[i] = pts.get(i).toVector();
            path = Paths.curve(v);
        }
        HeadingInterpolator h = s.heading;
        switch (h.type) {
            case CONSTANT:
                return path.constant(h.start);
            case LINEAR:
                return h.endTime >= 1 ? path.linear(h.start, h.end) : path.linear(h.start, h.end, h.endTime);
            case TANGENT:
            default:
                return h.reversed ? path.reverseTangent() : path.tangent();
        }
    }

    /*
     * Pedro core 3.0.0 bug: Line.pathCompletion(t) runs 1 -> 0, so LINEAR heading on a straight
     * line goes end -> start (fixed in 3.0.2). Checked once at runtime: when broken, straight
     * lines are built as 2-point Bezier curves, which are correct in every version.
     */
    private static final boolean LINE_COMPLETION_BROKEN =
            Paths.line(Vector2D.cartesian(0, 0), Vector2D.cartesian(10, 0)).curve.pathCompletion(0) > 0.5;

    private static double headingAtStart(Segment s) {
        switch (s.heading.type) {
            case CONSTANT:
            case LINEAR:
                return s.heading.start;
            case TANGENT:
            default:
                List<Pose> p = s.curve.getControlPoints();
                return tangent(p.get(0), p.get(1), s.heading.reversed);
        }
    }

    private static double headingAtEnd(Segment s) {
        switch (s.heading.type) {
            case CONSTANT:
                return s.heading.start;
            case LINEAR:
                return s.heading.end;
            case TANGENT:
            default:
                List<Pose> p = s.curve.getControlPoints();
                return tangent(p.get(p.size() - 2), p.get(p.size() - 1), s.heading.reversed);
        }
    }

    private static double tangent(Pose a, Pose b, boolean reversed) {
        double t = Math.atan2(b.getY() - a.getY(), b.getX() - a.getX());
        return reversed ? t + Math.PI : t;
    }
}
