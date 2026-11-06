package org.firstinspires.ftc.teamcode.pedro.imu;

import com.pedropathing.revhub.localizers.CustomIMU;
import com.pedropathing.revhub.localizers.RevHubIMU;
import com.pedropathing.tuning.autotune.DisplayName;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

/*
 * Picks the heading IMU for any Pedro localizer that takes a CustomIMU
 * (TwoWheelConfig, ThreeWheelIMUConfig).
 *
 *   c.imu.set(ImuSetup.navx().create());
 *   c.imu.set(ImuSetup.controlHub(LogoFacingDirection.UP, UsbFacingDirection.BACKWARD).create());
 *
 * Tuners use code() to print the matching line for Constants.
 */
public class ImuSetup {

    public enum Type {
        @DisplayName("navX Micro / navX2 Micro") NAVX,
        @DisplayName("Control Hub internal IMU") CONTROL_HUB
    }

    public final Type type;
    public final boolean navxReversed;
    public final RevHubOrientationOnRobot.LogoFacingDirection logoDirection;
    public final RevHubOrientationOnRobot.UsbFacingDirection usbDirection;

    public ImuSetup(
            Type type,
            boolean navxReversed,
            RevHubOrientationOnRobot.LogoFacingDirection logoDirection,
            RevHubOrientationOnRobot.UsbFacingDirection usbDirection
    ) {
        this.type = type;
        this.navxReversed = navxReversed;
        this.logoDirection = logoDirection;
        this.usbDirection = usbDirection;
    }

    public static ImuSetup navx() {
        return navx(false);
    }

    public static ImuSetup navx(boolean reversed) {
        return new ImuSetup(Type.NAVX, reversed,
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);
    }

    public static ImuSetup controlHub(
            RevHubOrientationOnRobot.LogoFacingDirection logoDirection,
            RevHubOrientationOnRobot.UsbFacingDirection usbDirection
    ) {
        return new ImuSetup(Type.CONTROL_HUB, false, logoDirection, usbDirection);
    }

    // new instance every call, each localizer needs its own IMU object
    public CustomIMU create() {
        if (type == Type.NAVX) {
            return new NavxIMU(navxReversed);
        }
        return new RevHubIMU(new RevHubOrientationOnRobot(logoDirection, usbDirection));
    }

    // Java for the c.imu.set(...) line in Constants
    public String code() {
        if (type == Type.NAVX) {
            return navxReversed ? "new NavxIMU(true)" : "new NavxIMU()";
        }
        return "new RevHubIMU(new RevHubOrientationOnRobot(\n" +
                "            RevHubOrientationOnRobot.LogoFacingDirection." + logoDirection.name() + ",\n" +
                "            RevHubOrientationOnRobot.UsbFacingDirection." + usbDirection.name() + "\n" +
                "    ))";
    }

    @Override
    public String toString() {
        if (type == Type.NAVX) {
            return "navX" + (navxReversed ? " (reversed)" : "");
        }
        return "Control Hub IMU (" + logoDirection + ", " + usbDirection + ")";
    }
}
