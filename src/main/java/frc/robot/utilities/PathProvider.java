package frc.robot.utilities;

import com.pathplanner.lib.path.PathPlannerPath;

import java.util.HashMap;
import java.util.Map;

public class PathProvider {
    private static final Map<String, PathPlannerPath> preloadedPaths = new HashMap<>();

    /**
     * Preload paths into memory.
     *
     * @param pathNames Array of path names to preload
     * @param constraints Default constraints to apply to all paths
     */
    public static void initialize(String[] pathNames) {
        for (String pathName : pathNames) {
            try {
                preloadedPaths.put(pathName, PathPlannerPath.fromPathFile(pathName));
                System.out.println("Path successfully loaded from file: " + pathName);
            } catch (Exception e) {
                System.err.println("Failed to load path: " + pathName);
                e.printStackTrace();
            }
        }
    }

    /**
     * Get a preloaded path by name.
     *
     * @param pathName The name of the path
     * @return The preloaded PathPlannerPath, or null if not found
     */
    public static PathPlannerPath fromPathFile(String pathName) {
        PathPlannerPath path = preloadedPaths.get(pathName);
    
        if (path == null) {
            System.err.println("Path not preloaded: " + pathName + ". Attempting to load from file...");
            try {
                path = PathPlannerPath.fromPathFile(pathName);
                preloadedPaths.put(pathName, path); // Cache the loaded path for future use
                System.out.println("Path successfully loaded from file: " + pathName);
            } catch (Exception e) {
                System.err.println("Failed to load path from file: " + pathName);
                e.printStackTrace();
            }
        } else {
            System.out.println("Path successfully loaded from cache: " + pathName);
        }
    
        return path;
    }
}