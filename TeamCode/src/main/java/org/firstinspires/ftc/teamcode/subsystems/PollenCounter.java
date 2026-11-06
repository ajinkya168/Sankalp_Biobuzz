package org.firstinspires.ftc.teamcode.subsystems;

import java.util.concurrent.atomic.AtomicInteger;

/*
 * Pollen counter - counts pieces passing a beam break. Plain Java, no FTC imports,
 * so this one file can be copied into another project (just change the package line).
 *
 *   one pollen = beam broken >= minBreakMs, then clear >= minClearMs (filters flicker)
 *   count reaches feederStopCount -> onFeederStop() feederStopDelayMs later
 *   count reaches maxCount        -> onFull() right away
 *   not counted while countingAllowed is false (shooting, reversing, full speed, ...)
 *
 * Call update() from one loop/thread as often as possible. reset(), getters and the
 * state flags are safe to use from other threads.
 *
 * Usage:
 *   PollenCounter pollen = new PollenCounter(PollenCounter.Params.fixed(10, 25, 3, 1000, 4),
 *           new PollenCounter.Listener() {
 *               public void onCount(int count) { gamepad.rumble(100); }
 *               public void onFeederStop()     { feeder.setPower(0); }
 *               public void onFull()           { intake.setPower(0); feeder.setPower(0); }
 *           });
 *   // in the loop:
 *   pollen.update(System.nanoTime(), beam.isBroken(), !shooting);
 */
public class PollenCounter {

    /** Thresholds, read every update() so live-tuned values (e.g. dashboard) apply at once. */
    public interface Params {
        double minBreakMs();
        double minClearMs();
        int feederStopCount();
        double feederStopDelayMs();
        int maxCount();

        /** Fixed values, for projects that don't tune them live. */
        static Params fixed(double minBreakMs, double minClearMs, int feederStopCount,
                            double feederStopDelayMs, int maxCount) {
            return new Params() {
                public double minBreakMs() { return minBreakMs; }
                public double minClearMs() { return minClearMs; }
                public int feederStopCount() { return feederStopCount; }
                public double feederStopDelayMs() { return feederStopDelayMs; }
                public int maxCount() { return maxCount; }
            };
        }
    }

    /** Called from the update() thread. Override only what you need. */
    public interface Listener {
        default void onCount(int count) {}
        default void onFeederStop() {}
        default void onFull() {}
    }

    private final Params params;
    private final Listener listener;

    private final AtomicInteger count = new AtomicInteger(0);
    private volatile boolean feederFull = false;  // feederStopCount reached and the delay ran out
    private volatile long feederStopAtNs = 0;     // pending delayed feeder stop, 0 = none
    private volatile boolean full = false;        // maxCount reached

    // beam debounce, only touched by update()
    private boolean beamPrev = false;
    private boolean pollenInBeam = false;
    private long brokenSinceNs = System.nanoTime();
    private long clearSinceNs = System.nanoTime();

    public PollenCounter(Params params, Listener listener) {
        this.params = params;
        this.listener = listener != null ? listener : new Listener() {};
    }

    /**
     * @param now             System.nanoTime()
     * @param broken          beam is broken right now
     * @param countingAllowed false = a new pollen in the beam is ignored
     */
    public void update(long now, boolean broken, boolean countingAllowed) {
        if (broken && !beamPrev) brokenSinceNs = now;
        if (!broken && beamPrev) clearSinceNs = now;
        double brokenMs = (now - brokenSinceNs) / 1e6;
        double clearMs = (now - clearSinceNs) / 1e6;

        if (broken && !pollenInBeam && brokenMs >= params.minBreakMs() && clearMs >= params.minClearMs()) {
            pollenInBeam = true;
            if (countingAllowed) {
                int c = count.incrementAndGet();
                listener.onCount(c);
                if (c >= params.feederStopCount() && !feederFull && feederStopAtNs == 0) {
                    feederStopAtNs = now + (long) (params.feederStopDelayMs() * 1e6);
                }
                if (c >= params.maxCount() && !full) {
                    full = true;
                    listener.onFull();
                }
            }
        }
        if (!broken && clearMs >= params.minClearMs()) pollenInBeam = false;
        beamPrev = broken;

        // delayed feeder stop
        long stopAt = feederStopAtNs;
        if (stopAt != 0 && now >= stopAt) {
            feederStopAtNs = 0;
            feederFull = true;
            listener.onFeederStop();
        }
    }

    /** Count back to 0, unlocks a full robot. */
    public void reset() {
        count.set(0);
        feederStopAtNs = 0;
        feederFull = false;
        full = false;
    }

    public int getCount() { return count.get(); }
    public boolean isFull() { return full; }
    public boolean isFeederFull() { return feederFull; }
    public boolean isFeederStopping() { return feederStopAtNs != 0; }
}
