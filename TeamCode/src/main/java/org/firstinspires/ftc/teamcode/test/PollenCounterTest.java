package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.subsystems.PollenCounter;

/*
 * Beam break pollen count test - ONLY beam breaks, no intake / feeder / IMU.
 * Push pollen through by hand and watch the count.
 *
 * ADDING / REMOVING A BEAM BREAK:
 *   1. Add / remove its name in BEAM_NAMES below, e.g. {"b1", "b2", "b3"}
 *   2. On the Driver Station: Configure Robot -> Control/Expansion Hub -> Digital Devices,
 *      set that port to "Digital Device" and give it the SAME name.
 *   That's it - every beam in the list gets its own counter and telemetry line.
 *
 * Each beam counts on its own:
 *   one pollen = beam broken >= MIN_BREAK_MS, then clear >= MIN_CLEAR_MS (filters flicker)
 *   rumble on every count, longer rumble when MAX_COUNT is reached
 *
 * Controls (gamepad1):
 *   B - reset all counts
 *
 * Tunable live on FTC Dashboard under "PollenCounterTest".
 * Dashboard graph: "<name> broken" (0/1) and "<name> count" for every beam.
 */
@Config
@TeleOp(name = "TEST: Pollen Counter", group = "Test")
public class PollenCounterTest extends OpMode {

    // ---- EDIT HERE: beam break names, must match the robot config ----
    public static final String[] BEAM_NAMES = {"b1", "b2"};

    public static boolean BEAM_BROKEN_WHEN_LOW = true;    // broken beam reads getState() == false
    public static double MIN_BREAK_MS = 10;               // shorter breaks are noise
    public static double MIN_CLEAR_MS = 25;               // beam clear this long before the next pollen
    public static int MAX_COUNT = 4;                      // "FULL" at this count
    public static long SLEEP_MS = 2;                      // sensor thread rest between polls

    private DigitalChannel[] beams;
    private PollenCounter[] counters;
    private volatile boolean[] broken;

    private volatile int rumbles = 0;              // counts seen by the thread, rumbled in loop()
    private volatile double hz = 0;
    private Thread thread;
    private volatile boolean threadRunning = false;
    private boolean bPrev = false;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        int n = BEAM_NAMES.length;
        beams = new DigitalChannel[n];
        counters = new PollenCounter[n];
        broken = new boolean[n];

        PollenCounter.Params params = new PollenCounter.Params() {
            public double minBreakMs() { return MIN_BREAK_MS; }
            public double minClearMs() { return MIN_CLEAR_MS; }
            public int feederStopCount() { return Integer.MAX_VALUE; } // no feeder in this test
            public double feederStopDelayMs() { return 0; }
            public int maxCount() { return MAX_COUNT; }
        };
        PollenCounter.Listener listener = new PollenCounter.Listener() {
            @Override
            public void onCount(int count) {
                rumbles++;
            }
        };

        for (int i = 0; i < n; i++) {
            beams[i] = hardwareMap.get(DigitalChannel.class, BEAM_NAMES[i]);
            beams[i].setMode(DigitalChannel.Mode.INPUT);
            counters[i] = new PollenCounter(params, listener);
        }

        telemetry.addLine("Pollen Counter Test ready - beams: " + String.join(", ", BEAM_NAMES));
        telemetry.addLine("B reset");
        telemetry.update();
    }

    @Override
    public void start() {
        threadRunning = true;
        thread = new Thread(this::watch, "PollenCounterTest");
        thread.start();
    }

    private void watch() {
        long windowStart = System.nanoTime();
        int polls = 0;
        while (threadRunning && !Thread.currentThread().isInterrupted()) {
            long now = System.nanoTime();
            for (int i = 0; i < beams.length; i++) {
                boolean b = BEAM_BROKEN_WHEN_LOW != beams[i].getState();
                broken[i] = b;
                counters[i].update(now, b, true);
            }

            polls++;
            if (now - windowStart >= 1_000_000_000L) {
                hz = polls * 1e9 / (now - windowStart);
                polls = 0;
                windowStart = now;
            }
            if (SLEEP_MS > 0) {
                try {
                    Thread.sleep(SLEEP_MS);
                } catch (InterruptedException e) {
                    return;
                }
            }
        }
    }

    @Override
    public void loop() {
        boolean b = gamepad1.b;
        if (b && !bPrev) {
            for (PollenCounter c : counters) c.reset();
        }
        bPrev = b;

        boolean anyFull = false;
        for (PollenCounter c : counters) anyFull |= c.isFull();
        if (rumbles > 0) {
            rumbles = 0;
            gamepad1.rumble(anyFull ? 500 : 100);
        }

        for (int i = 0; i < beams.length; i++) {
            String name = BEAM_NAMES[i];
            PollenCounter c = counters[i];
            telemetry.addData(name.toUpperCase() + " COUNT", "%d / %d%s",
                    c.getCount(), MAX_COUNT, c.isFull() ? "  FULL" : "");
            telemetry.addData(name + " broken", broken[i] ? 1 : 0);
            telemetry.addData(name + " count", c.getCount());
        }
        telemetry.addData("sensor thread Hz", "%.0f", hz);
        telemetry.addLine("B reset");
        telemetry.update();
    }

    @Override
    public void stop() {
        threadRunning = false;
        if (thread != null) {
            thread.interrupt();
            try {
                thread.join(500);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }
}
