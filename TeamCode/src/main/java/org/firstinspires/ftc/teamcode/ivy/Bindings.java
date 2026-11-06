package org.firstinspires.ftc.teamcode.ivy;

import com.pedropathing.ivy.Command;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

/**
 * A small joystick-binding registry built entirely on the Ivy {@code Command}/{@code Scheduler}
 * - replaces hand-rolled {@code boolean prevX} edge-detection flags without pulling in a second
 * command framework/scheduler.
 * <p>
 * Create exactly <b>one</b> {@code Bindings} per OpMode (as a field), register every button on
 * it once in {@code init()}, then call {@link #poll()} once per loop, alongside
 * {@code Scheduler.execute()}:
 * <pre>{@code
 * private final Bindings bindings = new Bindings();
 *
 * public void init() {
 *     Scheduler.reset();
 *     bindings.onTrue(() -> gamepad2.a, robot.shooter.fireOne());
 *     bindings.whileTrue(() -> gamepad1.right_bumper, robot.intake.in());
 *     bindings.toggleOnTrue(() -> gamepad2.right_bumper, robot.shooter.spinUpHeld(1500));
 * }
 *
 * public void loop() {
 *     bindings.poll();
 *     Scheduler.execute();
 * }
 * }</pre>
 */
public class Bindings {

    private final List<Entry> entries = new ArrayList<>();

    /** Schedules {@code command} once, the loop {@code condition} becomes true (like "whenPressed"). */
    public Bindings onTrue(BooleanSupplier condition, Command command) {
        entry(condition).onTrueActions.add(command::schedule);
        return this;
    }

    /** Schedules {@code command} once, the loop {@code condition} becomes false (like "whenReleased"). */
    public Bindings onFalse(BooleanSupplier condition, Command command) {
        entry(condition).onFalseActions.add(command::schedule);
        return this;
    }

    /** Schedules {@code command} while {@code condition} is true, and cancels it the moment it goes false (like "whenHeld"). */
    public Bindings whileTrue(BooleanSupplier condition, Command command) {
        Entry entry = entry(condition);
        entry.onTrueActions.add(command::schedule);
        entry.onFalseActions.add(command::cancel);
        return this;
    }

    /** Each time {@code condition} becomes true, schedules {@code command} if it isn't running, or cancels it if it is (like "toggleWhenPressed"). */
    public Bindings toggleOnTrue(BooleanSupplier condition, Command command) {
        entry(condition).onTrueActions.add(() -> {
            if (command.isScheduled()) {
                command.cancel();
            } else {
                command.schedule();
            }
        });
        return this;
    }

    /** Polls every condition registered on this Bindings instance. Call exactly once per loop. */
    public void poll() {
        for (Entry entry : entries) {
            entry.poll();
        }
    }

    private Entry entry(BooleanSupplier condition) {
        Entry entry = new Entry(condition);
        entries.add(entry);
        return entry;
    }

    private static final class Entry {
        final BooleanSupplier condition;
        final List<Runnable> onTrueActions = new ArrayList<>();
        final List<Runnable> onFalseActions = new ArrayList<>();
        boolean previous = false;

        Entry(BooleanSupplier condition) {
            this.condition = condition;
        }

        void poll() {
            boolean current = condition.getAsBoolean();
            if (current && !previous) onTrueActions.forEach(Runnable::run);
            if (!current && previous) onFalseActions.forEach(Runnable::run);
            previous = current;
        }
    }
}
