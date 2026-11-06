package org.firstinspires.ftc.teamcode.hdware;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/*
 * Velocity PIDF for a flywheel (ticks/s in, motor power out):
 *   power = kP*error + kI*integral(error dt) + kD*d(error)/dt + kF*target
 * Gains are passed in on every update() so they can be tuned live on FTC Dashboard and
 * swapped between game pieces (pollen / nectar) without making a new controller.
 * The integral term is clamped to +-iMax power (anti-windup). Call reset() when the
 * target or the gain set changes, and whenever the shooter is turned off.
 * The last P / I / D / F terms are kept for telemetry.
 *
 * Same as the sankalp27 VelocityPIDF.
 */
public class VelocityPIDF {

    private final ElapsedTime timer = new ElapsedTime();
    private double integral = 0;
    private double lastError = 0;
    private boolean first = true;

    public double pTerm = 0, iTerm = 0, dTerm = 0, fTerm = 0, error = 0;

    public void reset() {
        integral = 0;
        lastError = 0;
        first = true;
        pTerm = iTerm = dTerm = fTerm = error = 0;
    }

    public double update(double target, double measured,
                         double kP, double kI, double kD, double kF, double iMax) {
        double dt = timer.seconds();
        timer.reset();

        error = target - measured;
        if (first) { // no history yet - no I / D on the first step
            dt = 0;
            lastError = error;
            first = false;
        }

        if (dt > 0) integral += error * dt;
        if (kI != 0) integral = Range.clip(integral, -iMax / Math.abs(kI), iMax / Math.abs(kI));
        double derivative = dt > 0 ? (error - lastError) / dt : 0;
        lastError = error;

        pTerm = kP * error;
        iTerm = kI * integral;
        dTerm = kD * derivative;
        fTerm = kF * target;
        return pTerm + iTerm + dTerm + fTerm;
    }
}
