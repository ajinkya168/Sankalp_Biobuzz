package org.firstinspires.ftc.teamcode.opmodes.auto;

import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.auto.paths.ExamplePaths;
import org.firstinspires.ftc.teamcode.pedro.visualizer.VisualizerPaths;

import java.util.ArrayList;
import java.util.List;

/*
 * ============================== BASIC AUTO ==============================
 *
 * TO TEST A NEW PATH: paste the visualizer export into opmodes/auto/paths/ (README.md),
 * then change the ONE line below:      PATHS = YourNewPaths.class
 *
 * What happens on each path (PLAN):
 *   ""  (empty) = default pattern, works for any number of paths:
 *        path 1, 3, 5 ... -> SCORE   drive there while the shooter spins up, then shoot
 *        path 2, 4, 6 ... -> COLLECT intake on while driving
 *        last path, if it is a "collect" one -> PARK (everything off) - nothing to shoot after it
 *   or one letter per path, e.g. "SCSCP":
 *        S = score    C = collect    D = drive only    P = park
 *
 * SHOOT_PRELOAD_FIRST = true -> shoots the preload BEFORE the first path (robot starts
 * where it can already shoot).
 */
@Autonomous(name = "Basic Auto (Visualizer)", group = "Final", preselectTeleOp = "Final Command TeleOp 2 (Ivy)")
public class BasicAuto extends BaseAuto {

    // ===================== CHANGE THESE =====================
    private static final Class<?> PATHS = ExamplePaths.class;
    private static final String PLAN = "";
    private static final boolean SHOOT_PRELOAD_FIRST = false;
    // ========================================================

    @Override
    protected Class<?> pathsClass() {
        return PATHS;
    }

    @Override
    protected Command routine(VisualizerPaths p, AutoActions a) {
        List<Command> steps = new ArrayList<>();
        if (SHOOT_PRELOAD_FIRST) steps.add(sequential(a.label("shoot preload"), a.shoot()));

        for (int i = 0; i < p.size(); i++) {
            String name = p.name(i);
            switch (stepFor(i, p.size())) {
                case 'S':
                    steps.add(a.scoreAt(name, p.get(i)));
                    break;
                case 'C':
                    steps.add(a.collect(name, p.get(i)));
                    break;
                case 'P':
                    steps.add(a.park(name, p.get(i)));
                    break;
                case 'D':
                default:
                    steps.add(a.drive(name, p.get(i)));
                    break;
            }
        }
        return sequential(steps.toArray(new Command[0]));
    }

    /** Letter for path i: from PLAN, or the default score / collect pattern. */
    private static char stepFor(int i, int count) {
        if (i < PLAN.length()) return Character.toUpperCase(PLAN.charAt(i));
        if (!PLAN.isEmpty()) return 'D';                       // PLAN shorter than the path list
        boolean score = SHOOT_PRELOAD_FIRST ? i % 2 == 1 : i % 2 == 0;
        if (!score && i == count - 1) return 'P';             // collecting last = nothing to shoot -> park
        return score ? 'S' : 'C';
    }
}
