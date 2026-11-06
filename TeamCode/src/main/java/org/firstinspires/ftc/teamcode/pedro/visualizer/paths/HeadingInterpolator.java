package org.firstinspires.ftc.teamcode.pedro.visualizer.paths;

/** Visualizer-export stand-in for the Pedro 2.x HeadingInterpolator (radians). */
public final class HeadingInterpolator {

    public enum Type { TANGENT, CONSTANT, LINEAR }

    public final Type type;
    public final double start, end, endTime;
    public final boolean reversed;

    private HeadingInterpolator(Type type, double start, double end, double endTime, boolean reversed) {
        this.type = type;
        this.start = start;
        this.end = end;
        this.endTime = endTime;
        this.reversed = reversed;
    }

    public static final HeadingInterpolator tangent = new HeadingInterpolator(Type.TANGENT, 0, 0, 1, false);

    public static HeadingInterpolator constant(double heading) {
        return new HeadingInterpolator(Type.CONSTANT, heading, heading, 1, false);
    }

    public static HeadingInterpolator linear(double start, double end) {
        return linear(start, end, 1);
    }

    public static HeadingInterpolator linear(double start, double end, double endTime) {
        return new HeadingInterpolator(Type.LINEAR, start, end, endTime, false);
    }

    public HeadingInterpolator reverse() {
        return withReversed(!reversed);
    }

    HeadingInterpolator withReversed(boolean r) {
        return new HeadingInterpolator(type, start, end, endTime, r);
    }
}
