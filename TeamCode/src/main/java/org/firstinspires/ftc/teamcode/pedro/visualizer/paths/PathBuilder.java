package org.firstinspires.ftc.teamcode.pedro.visualizer.paths;

import org.firstinspires.ftc.teamcode.pedro.visualizer.geometry.BezierCurve;

import java.util.ArrayList;
import java.util.List;

/**
 * Visualizer-export stand-in for the Pedro 2.x PathBuilder. Heading setters apply to the
 * LAST path added (same as Pedro 2.x); a path with no heading set follows its tangent.
 * Every {@link #build()} is recorded in order, so the auto runs the chains in the order the
 * visualizer wrote them.
 */
public class PathBuilder {

    /** Chains in the order they were built - read (and cleared) by VisualizerPaths. */
    public static final List<PathChain> BUILT = new ArrayList<>();

    private final List<BezierCurve> curves = new ArrayList<>();
    private final List<HeadingInterpolator> headings = new ArrayList<>();

    public PathBuilder addPath(BezierCurve curve) {
        curves.add(curve);
        headings.add(HeadingInterpolator.tangent);
        return this;
    }

    public PathBuilder addPaths(BezierCurve... curves) {
        for (BezierCurve c : curves) addPath(c);
        return this;
    }

    public PathBuilder setHeadingInterpolation(HeadingInterpolator h) {
        boolean reversed = last().reversed;
        return setLast(h.reversed ? h : h.withReversed(reversed));
    }

    public PathBuilder setLinearHeadingInterpolation(double start, double end) {
        return setHeadingInterpolation(HeadingInterpolator.linear(start, end));
    }

    public PathBuilder setLinearHeadingInterpolation(double start, double end, double endTime) {
        return setHeadingInterpolation(HeadingInterpolator.linear(start, end, endTime));
    }

    public PathBuilder setConstantHeadingInterpolation(double heading) {
        return setHeadingInterpolation(HeadingInterpolator.constant(heading));
    }

    public PathBuilder setTangentHeadingInterpolation() {
        return setHeadingInterpolation(HeadingInterpolator.tangent);
    }

    /** Drive the last path backwards (tangent heading + 180 deg). */
    public PathBuilder setReversed() {
        return setReversed(true);
    }

    public PathBuilder setReversed(boolean reversed) {
        return setLast(last().withReversed(reversed));
    }

    public PathChain build() {
        List<PathChain.Segment> segs = new ArrayList<>();
        for (int i = 0; i < curves.size(); i++) segs.add(new PathChain.Segment(curves.get(i), headings.get(i)));
        PathChain chain = new PathChain(segs);
        BUILT.add(chain);
        return chain;
    }

    private HeadingInterpolator last() {
        if (headings.isEmpty()) throw new IllegalStateException("addPath(...) before setting a heading");
        return headings.get(headings.size() - 1);
    }

    private PathBuilder setLast(HeadingInterpolator h) {
        last();
        headings.set(headings.size() - 1, h);
        return this;
    }
}
