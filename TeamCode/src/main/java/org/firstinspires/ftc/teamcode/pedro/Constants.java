package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.Encoder;
import com.pedropathing.revhub.localizers.TwoWheelConfig;
import com.pedropathing.revhub.localizers.TwoWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedro.imu.NavxIMU;

/*
 * Pedro Pathing 3.0 constants for the Final robot.
 *
 * Drivetrain - Mecanum, motor names from FinalHardware (fl, fr, bl, br)
 * Localizer  - Two Wheel (forward + strafe dead wheel) with navX heading
 * Algorithm  - Foresight
 *
 * Tuning order (Tuning OpMode on the Driver Hub / Panels):
 *   1. Mecanum Tuner    -> paste into drivetrainConfig
 *   2. Two Wheel Tuner  -> paste into localizerConfig (keep c.imu.set(new NavxIMU()))
 *   3. Foresight Tuner  -> paste into foresightConfig
 *   4. Tests            -> Localization / Line / Curve / Hold to check
 *
 * Everything marked DEFAULT is a starting value only, it must be replaced by the tuner.
 */
public class Constants {

    public static Follower create(HardwareMap h) {
        return new Follower(localizer(h), drivetrain(h), algorithm());
    }

    public static Drivetrain drivetrain(HardwareMap h) {
        return new Mecanum(h, drivetrainConfig);
    }

    public static Localizer localizer(HardwareMap h) {
        return new TwoWheelLocalizer(h, localizerConfig);
    }

    public static Foresight algorithm() {
        return new Foresight(foresightConfig);
    }

    //TODO Drivetrain (same names and directions as the old FinalHardware)
    public static MecanumConfig drivetrainConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set("fl");
                c.frontRightName.set("fr");
                c.backLeftName.set("bl");
                c.backRightName.set("br");

                c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
                c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
                c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
                c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);

                c.manualBrakeMode.set(true);
            }
    );

    //TODO Localizer - Two Wheel + navX
    // Dead wheels are read through the encoder port of a motor, so the pod names are the
    // motor names whose encoder port each pod is plugged into. Change these to match the wiring.
    public static TwoWheelConfig localizerConfig = new TwoWheelConfig(c -> {
        c.xPodName.set("br");
        c.yPodName.set("fr");
        c.imuName.set("navx");
        c.xPodOffset.set(0.9189408513048847);
        c.yPodOffset.set(-0.5650023368060171);
        c.forwardTicksToInches.set(7.266097973807163E-4);
        c.strafeTicksToInches.set(7.348061601771524E-4);
        c.xPodDirection.set(Encoder.REVERSE);
        c.yPodDirection.set(Encoder.FORWARD);
        c.imu.set(new NavxIMU());
    });
    //TODO Foresight - replace the whole block with the Foresight Tuner output
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.3635764061439184);
                Controller secondaryTranslationalForward = Controller.proportional(0.1343317671296831);
                Controller primaryTranslationalLateral = Controller.proportional(0.8827262624024651);
                Controller secondaryTranslationalLateral = Controller.proportional(0.3261437670775736);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.011400231259624999));
                c.brake.set(Controller.proportionalFeedforward(0.009690196570681249));

                c.headingFeedback.set(Controller.proportional(21.410853540429898));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.0027045008468330025, 1.5165360685595326E-4));

                c.linearBrakeCoefficients.set(Matrix.diag(6.594915885871279, 1.6504699400550562));
                c.quadraticBrakeCoefficients.set(Matrix.diag(-0.10013129744531817, -0.0275540210656814));

                c.maxAchievableForwardVelocity.set(70.83809722124377);
                c.maxAchievableStrafeVelocity.set(52.782339811242345);
                c.naturalForwardDeceleration.set(28.91974292356384);
                c.naturalStrafeDeceleration.set(96.74100677350776);
            }
    );
}
