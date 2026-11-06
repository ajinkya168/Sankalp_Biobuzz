package org.firstinspires.ftc.teamcode.pedro.visualizer;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.pedro.visualizer.follower.Follower;
import org.firstinspires.ftc.teamcode.pedro.visualizer.paths.PathBuilder;
import org.firstinspires.ftc.teamcode.pedro.visualizer.paths.PathChain;

import java.lang.reflect.Constructor;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.Collections;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;

/*
 * ============================== VISUALIZER PATHS ==============================
 *
 * Loads the class exported by the Pedro Pathing visualizer and turns it into Pedro 3.0 paths.
 *
 *   VisualizerPaths paths = VisualizerPaths.load(MyExportedPaths.class);
 *   paths.start()       where to place the robot (start of the first path, with heading)
 *   paths.get(0)        first path, in the order the visualizer wrote them
 *   paths.get("Path3")  a path by its field name
 *
 * Accepted shapes of the exported class (pass the outer class, nested classes are searched):
 *   1. public static class Path1 { public PathChain Path1; ... public Path1(Follower follower) {...} }
 *   2. a class with a no-arg constructor or static PathChain fields built with new PathBuilder()
 *   3. a class with public com.pedropathing.paths.Path fields (native Pedro 3.0 code) and an
 *      optional com.pedropathing.math.Pose field with "start" in its name
 *
 * For shapes 1 + 2 the export's imports must point at this package, see opmodes/auto/README.md.
 */
public final class VisualizerPaths {

    private final List<String> names;
    private final List<Path> paths;
    private final Pose start;

    private VisualizerPaths(List<String> names, List<Path> paths, Pose start) {
        if (paths.isEmpty()) throw new IllegalStateException("No paths found in the exported class");
        this.names = Collections.unmodifiableList(names);
        this.paths = Collections.unmodifiableList(paths);
        this.start = start;
    }

    public Pose start() { return start; }

    public int size() { return paths.size(); }

    public Path get(int index) { return paths.get(index); }

    public String name(int index) { return names.get(index); }

    public List<String> names() { return names; }

    public Path get(String name) {
        int i = names.indexOf(name);
        if (i < 0) throw new IllegalArgumentException("No path named " + name + " - have " + names);
        return paths.get(i);
    }

    // =====================================================================================
    //                                      LOADING
    // =====================================================================================

    public static VisualizerPaths load(Class<?> exported) {
        List<Class<?>> candidates = new ArrayList<>();
        candidates.add(exported);
        Collections.addAll(candidates, exported.getDeclaredClasses());

        // 1. a class with a (Follower) constructor - the current visualizer export
        for (Class<?> c : candidates) {
            Constructor<?> ctor = constructor(c, Follower.class);
            if (ctor != null) return fromChains(c, newInstance(ctor, new Follower()));
        }
        // 2. no-arg constructor / static fields
        for (Class<?> c : candidates) {
            if (!hasFields(c, PathChain.class) && !hasFields(c, Path.class)) continue;
            Constructor<?> ctor = constructor(c);
            Object instance = ctor != null ? newInstance(ctor) : null;
            return hasFields(c, PathChain.class) ? fromChains(c, instance) : fromNative(c, instance);
        }
        throw new IllegalArgumentException(exported.getSimpleName()
                + " has no PathChain / Path fields - is it the class exported by the visualizer?");
    }

    // PathChain fields, run in the order build() was called (falls back to field-name order)
    private static VisualizerPaths fromChains(Class<?> c, Object instance) {
        List<PathChain> order = new ArrayList<>(PathBuilder.BUILT);
        PathBuilder.BUILT.clear();

        Map<PathChain, String> fieldNames = new IdentityHashMap<>();
        List<Field> fields = fields(c, PathChain.class);
        for (Field f : fields) {
            PathChain chain = (PathChain) read(f, instance);
            if (chain != null) fieldNames.put(chain, f.getName());
        }
        // static fields only build once per app run, so a re-run sees nothing in BUILT
        if (order.isEmpty()) {
            for (Field f : fields) {
                PathChain chain = (PathChain) read(f, instance);
                if (chain != null) order.add(chain);
            }
        }

        List<String> names = new ArrayList<>();
        List<Path> paths = new ArrayList<>();
        for (int i = 0; i < order.size(); i++) {
            PathChain chain = order.get(i);
            String n = fieldNames.get(chain);
            names.add(n != null ? n : "path" + (i + 1));
            paths.add(chain.toPath());
        }
        Pose start = order.isEmpty() ? Pose.zero() : order.get(0).startPose().toPedro();
        return new VisualizerPaths(names, paths, start);
    }

    // native Pedro 3.0 Path fields, in field-name order
    private static VisualizerPaths fromNative(Class<?> c, Object instance) {
        List<String> names = new ArrayList<>();
        List<Path> paths = new ArrayList<>();
        for (Field f : fields(c, Path.class)) {
            Path p = (Path) read(f, instance);
            if (p == null) continue;
            names.add(f.getName());
            paths.add(p);
        }
        Pose start = null;
        for (Field f : fields(c, Pose.class)) {
            if (f.getName().toLowerCase().contains("start")) start = (Pose) read(f, instance);
        }
        if (start == null && !paths.isEmpty()) start = paths.get(0).get(0);
        return new VisualizerPaths(names, paths, start);
    }

    // =====================================================================================
    //                                 REFLECTION HELPERS
    // =====================================================================================

    private static Constructor<?> constructor(Class<?> c, Class<?>... params) {
        if (Modifier.isAbstract(c.getModifiers())) return null;
        if (c.isMemberClass() && !Modifier.isStatic(c.getModifiers())) return null;  // inner class needs an outer instance
        try {
            Constructor<?> ctor = c.getDeclaredConstructor(params);
            ctor.setAccessible(true);
            return ctor;
        } catch (NoSuchMethodException e) {
            return null;
        }
    }

    private static Object newInstance(Constructor<?> ctor, Object... args) {
        try {
            return ctor.newInstance(args);
        } catch (ReflectiveOperationException e) {
            Throwable cause = e.getCause() != null ? e.getCause() : e;
            throw new RuntimeException("Could not build " + ctor.getDeclaringClass().getSimpleName() + ": " + cause, cause);
        }
    }

    private static boolean hasFields(Class<?> c, Class<?> type) {
        return !fields(c, type).isEmpty();
    }

    /** Fields of the given type, sorted by name with numbers in natural order (Path2 before Path10). */
    private static List<Field> fields(Class<?> c, Class<?> type) {
        List<Field> out = new ArrayList<>();
        for (Field f : c.getDeclaredFields()) {
            if (type.isAssignableFrom(f.getType())) {
                f.setAccessible(true);
                out.add(f);
            }
        }
        Collections.sort(out, (a, b) -> naturalCompare(a.getName(), b.getName()));
        return out;
    }

    private static Object read(Field f, Object instance) {
        if (instance == null && !Modifier.isStatic(f.getModifiers())) return null;
        try {
            return f.get(instance);
        } catch (IllegalAccessException e) {
            return null;
        }
    }

    private static int naturalCompare(String a, String b) {
        int i = 0, j = 0;
        while (i < a.length() && j < b.length()) {
            char ca = a.charAt(i), cb = b.charAt(j);
            if (Character.isDigit(ca) && Character.isDigit(cb)) {
                int si = i, sj = j;
                while (i < a.length() && Character.isDigit(a.charAt(i))) i++;
                while (j < b.length() && Character.isDigit(b.charAt(j))) j++;
                long na = Long.parseLong(a.substring(si, Math.min(i, si + 18)));
                long nb = Long.parseLong(b.substring(sj, Math.min(j, sj + 18)));
                if (na != nb) return Long.compare(na, nb);
            } else {
                if (ca != cb) return Character.compare(ca, cb);
                i++;
                j++;
            }
        }
        return Integer.compare(a.length() - i, b.length() - j);
    }
}
