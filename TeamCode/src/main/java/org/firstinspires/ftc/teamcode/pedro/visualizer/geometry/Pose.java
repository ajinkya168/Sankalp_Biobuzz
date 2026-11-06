package org.firstinspires.ftc.teamcode.pedro.visualizer.geometry;

/**
 * Visualizer-export stand-in for the Pedro 2.x {@code com.pedropathing.geometry.Pose}.
 * Field inches, heading in radians. Converted to a Pedro 3.0 {@link com.pedropathing.math.Pose} by {@link #toPedro()}.
 */
public class Pose {
    private final double x, y, heading;

    public Pose(double x, double y) {
        this(x, y, 0);
    }

    public Pose(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public double getX() { return x; }
    public double getY() { return y; }
    public double getHeading() { return heading; }

    public Pose withHeading(double heading) {
        return new Pose(x, y, heading);
    }

    public com.pedropathing.math.Pose toPedro() {
        return new com.pedropathing.math.Pose(x, y, heading);
    }

    public com.pedropathing.math.Vector2D toVector() {
        return com.pedropathing.math.Vector2D.cartesian(x, y);
    }

    @Override
    public String toString() {
        return String.format("(%.1f, %.1f, %.0f deg)", x, y, Math.toDegrees(heading));
    }
}
