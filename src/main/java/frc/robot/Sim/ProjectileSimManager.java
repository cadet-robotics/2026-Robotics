package frc.robot.Sim;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;
import java.lang.reflect.Method;
import java.lang.reflect.InvocationTargetException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Lightweight projectile simulation manager for simulation mode.
 *
 * Publishes projectile positions as an array of x,y pairs (meters) to
 * "Sim/Projectiles" so external visualizers (AdvantageScope) can render them.
 */
public class ProjectileSimManager {
    private static ProjectileSimManager instance = new ProjectileSimManager();

    public static ProjectileSimManager getInstance() { return instance; }

    private Supplier<Pose2d> robotPoseSupplier = null;

    public void setRobotPoseSupplier(Supplier<Pose2d> s) { this.robotPoseSupplier = s; }

    private final List<Projectile> projectiles = new ArrayList<>();
    // If MapleSim is available we'll create entities there and keep handles here.
    private final Map<Projectile, Object> mapleEntities = new HashMap<>();
    private boolean mapleAvailable = false;
    private Object mapleArenaInstance = null;
    private Method mapleCreateMethod = null; // candidate to create a ball/entity
    // common accessor names we'll try when reading back pose from a Maple entity
    private static final String[] POS_X_METHODS = new String[] {"getX", "x", "getPosX", "getPositionX", "posX", "getPoseX"};
    private static final String[] POS_Y_METHODS = new String[] {"getY", "y", "getPosY", "getPositionY", "posY", "getPoseY"};
    private static final String[] POS_Z_METHODS = new String[] {"getZ", "z", "getPosZ", "getPositionZ", "posZ", "getPoseZ"};
    // totalSpawned removed — not used when not publishing to NetworkTables

    private ProjectileSimManager() {}

    {
        // no NetworkTables publishing for projectiles anymore; use MapleSim entities for visualization
    }

    // Reflection-based MapleSim discovery (runs once at class init)
    static {
        // nothing here; instance initializer below initializes Maple availability per-instance
    }

    {
        // Try to discover MapleSim (IronMaple) arena via reflection so we don't hard compile against
        // vendor types. If discovery fails we silently fall back to the local lightweight sim.
        try {
            // Try known package locations for SimulatedArena. Some vendordeps/wrappers use
            // `swervelib.simulation.ironmaple.simulation.SimulatedArena` while upstream uses
            // `org.ironmaple.simulation.SimulatedArena`.
            Class<?> arenaClass = null;
            try {
                arenaClass = Class.forName("swervelib.simulation.ironmaple.simulation.SimulatedArena");
            } catch (ClassNotFoundException e) {
                arenaClass = Class.forName("org.ironmaple.simulation.SimulatedArena");
            }
            Method getInst = arenaClass.getMethod("getInstance");
            mapleArenaInstance = getInst.invoke(null);

            // Try several likely factory method names that create a projectile entity. Many Maple
            // installations expose a convenience method taking (double x,double y,double z,double vx,double vy,double vz)
            String[] createNames = new String[] {"createBall", "spawnBall", "addBall", "createProjectile", "spawnProjectile", "addProjectile", "createEntity", "spawnEntity", "addEntity"};
            for (String name : createNames) {
                try {
                    mapleCreateMethod = arenaClass.getMethod(name, double.class, double.class, double.class, double.class, double.class, double.class);
                    break;
                } catch (NoSuchMethodException e) {
                    // try next
                }
            }

            // If we found an arena instance, consider Maple available. We'll only spawn when we
            // have a usable create method; otherwise we intentionally do NOT fall back to local physics.
        mapleAvailable = mapleArenaInstance != null;
        } catch (ClassNotFoundException | IllegalAccessException | InvocationTargetException | NoSuchMethodException ex) {
            // MapleSim not present or unexpected API; we will not run local physics anymore — we
            // rely on Maple to create projectiles. Keep mapleAvailable false so spawns are skipped.
            mapleAvailable = false;
            mapleArenaInstance = null;
            mapleCreateMethod = null;
        }
        // MapleSim availability determined; logging removed to save disk space
    }

    /** Spawn a projectile from a given pose. yaw is robot heading (radians). */
    public void spawnProjectile(Pose2d startPose, double muzzleSpeedMetersPerSecond, double yawRadians, double elevationRadians) {
        // Start a bit in front of the robot
        double startX = startPose.getX() + 0.2 * Math.cos(yawRadians);
        double startY = startPose.getY() + 0.2 * Math.sin(yawRadians);
        double startZ = 0.9; // meters above ground (approx shooter height)

        double v = muzzleSpeedMetersPerSecond;
        double vx = v * Math.cos(elevationRadians) * Math.cos(yawRadians);
        double vy = v * Math.cos(elevationRadians) * Math.sin(yawRadians);
        double vz = v * Math.sin(elevationRadians);

        // We no longer run our own projectile physics. If MapleSim is present and exposes a
        // factory method taking (x,y,z,vx,vy,vz) we'll call it to create a Maple-managed projectile.
        if (!mapleAvailable || mapleCreateMethod == null || mapleArenaInstance == null) {
            // MapleSim not available — skip spawning projectiles
            return;
        }

        try {
            Object mapleEntity = mapleCreateMethod.invoke(mapleArenaInstance, startX, startY, startZ, vx, vy, vz);
            if (mapleEntity != null) {
                Projectile p = new Projectile(startX, startY, startZ, vx, vy, vz);
                synchronized (projectiles) {
                    projectiles.add(p);
                    mapleEntities.put(p, mapleEntity);
                }
            } else {
                // Maple create method returned null; skip
            }
        } catch (IllegalAccessException | InvocationTargetException ex) {
            // Invocation failed; skip spawn
        }
    }

    /** Convenience spawn using the registered robot pose supplier. */
    public void spawnProjectileFromRobot(double muzzleSpeedMetersPerSecond, double elevationRadians) {
        if (robotPoseSupplier == null) return;
        Pose2d pose = robotPoseSupplier.get();
        spawnProjectile(pose, muzzleSpeedMetersPerSecond, pose.getRotation().getRadians(), elevationRadians);
    }

    /** Step the simulation by dt seconds (call from simulationPeriodic). */
    public void update(double dt) {
        synchronized (projectiles) {
            Iterator<Projectile> it = projectiles.iterator();
            while (it.hasNext()) {
                Projectile p = it.next();
                Object mapleEntity = mapleEntities.get(p);
                if (mapleEntity == null) {
                    // We intentionally do not run local physics anymore. If the Maple entity is
                    // missing something went wrong; remove the projectile from local bookkeeping.
                    it.remove();
                    mapleEntities.remove(p);
                    continue;
                }

                // Read position from Maple entity via reflection.
                try {
                    Double x = tryGetDoubleFromEntity(mapleEntity, POS_X_METHODS);
                    Double y = tryGetDoubleFromEntity(mapleEntity, POS_Y_METHODS);
                    Double z = tryGetDoubleFromEntity(mapleEntity, POS_Z_METHODS);
                    if (x != null && y != null && z != null) {
                        p.x = x;
                        p.y = y;
                        p.z = z;
                    }
                    p.age += dt;
                    // If MapleSim moved it below ground we remove it locally as well
                    if (p.z <= 0.0 || p.age > 30.0) {
                        it.remove();
                        mapleEntities.remove(p);
                    }
                } catch (Exception ex) {
                    // couldn't read Maple entity; remove local bookkeeping
                    mapleEntities.remove(p);
                    it.remove();
                }
            }

            // We no longer publish projectile state to NetworkTables under "Sim/*".
            // Visualization should be done via MapleSim entities or other tools.
        }
    }

    /**
     * Return a copy of current projectile poses as Pose3d objects.
     */
    public List<Pose3d> getProjectilePose3dList() {
        List<Pose3d> out = new ArrayList<>();
        synchronized (projectiles) {
            for (Projectile p : projectiles) {
                // Build a simple rotation based on velocity if we have it; otherwise keep zero.
                double yaw = 0.0;
                try {
                    yaw = Math.atan2(p.vy, p.vx);
                } catch (Exception e) {
                    yaw = 0.0;
                }
                out.add(new Pose3d(new Translation3d(p.x, p.y, Math.max(p.z, 0.0)), new Rotation3d(0.0, 0.0, yaw)));
            }
        }
        return out;
    }

    private static class Projectile {
        double x, y, z;
        double vx, vy;
        double age = 0.0;

        Projectile(double x, double y, double z, double vx, double vy, double vz) {
            this.x = x; this.y = y; this.z = z;
            this.vx = vx; this.vy = vy; 
        }
    }

    /**
     * Try to extract a double value from a Maple entity by trying a list of common method/field names.
     * Returns null if none found or an error occurs.
     */
    private Double tryGetDoubleFromEntity(Object entity, String[] candidateNames) {
        if (entity == null) return null;
        Class<?> cls = entity.getClass();
        // If the entity exposes getPose3d(), prefer that — it's the most robust
        try {
            Method getPose = cls.getMethod("getPose3d");
            Object poseObj = getPose.invoke(entity);
            if (poseObj != null) {
                Class<?> poseCls = poseObj.getClass();
                try {
                    Method mx = poseCls.getMethod("getX");
                    Object ox = mx.invoke(poseObj);
                    if (ox instanceof Number) return ((Number) ox).doubleValue();
                } catch (Exception e) {
                    // fallthrough to other checks
                }
            }
        } catch (NoSuchMethodException | IllegalAccessException | InvocationTargetException ex) {
            // ignore
        }
        for (String name : candidateNames) {
            try {
                // try method first
                Method m = cls.getMethod(name);
                Object val = m.invoke(entity);
                if (val instanceof Number) return ((Number) val).doubleValue();
            } catch (NoSuchMethodException nsme) {
                // try field
                try {
                    java.lang.reflect.Field f = cls.getField(name);
                    Object val = f.get(entity);
                    if (val instanceof Number) return ((Number) val).doubleValue();
                } catch (NoSuchFieldException | IllegalAccessException ex) {
                    // ignore and try next name
                }
            } catch (IllegalAccessException | InvocationTargetException ex) {
                // ignore and try next candidate
            }
        }
        // Try to see if there's a generic getPosition/getPose method returning an object with x/y/z
        try {
            Method getPos = cls.getMethod("getPosition");
            Object posObj = getPos.invoke(entity);
            if (posObj != null) {
                Class<?> posCls = posObj.getClass();
                try {
                    Method mx = posCls.getMethod("getX");
                    Object vx = mx.invoke(posObj);
                    if (vx instanceof Number) return ((Number) vx).doubleValue();
                } catch (Exception e) {
                    // fallthrough
                }
            }
        } catch (NoSuchMethodException | IllegalAccessException | InvocationTargetException e) {
            // ignore
        }
        return null;
    }
}
