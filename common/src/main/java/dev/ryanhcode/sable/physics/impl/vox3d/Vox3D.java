package dev.ryanhcode.sable.physics.impl.vox3d;

import dev.ryanhcode.sable.api.physics.PhysicsPipelineBody;
import dev.ryanhcode.sable.api.physics.callback.BlockSubLevelCollisionCallback;
import dev.ryanhcode.sable.api.physics.mass.MassData;
import dev.ryanhcode.sable.physics.impl.vox3d.collider.Vox3DVoxelColliderData;
import org.jetbrains.annotations.Nullable;
import org.joml.Matrix3dc;
import org.joml.Vector3dc;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.File;
import java.io.FileOutputStream;
import java.io.InputStream;
import java.nio.file.Files;
import java.util.concurrent.atomic.AtomicInteger;

/**
 * Native bindings and JNI wrapper for the Vox3D (Box3D) C physics engine.
 */
public final class Vox3D {

    public static final String NATIVE_NAME = "sable_vox3d";
    private static final Logger LOGGER = LoggerFactory.getLogger(Vox3D.class);
    private static final AtomicInteger BODY_ID_COUNTER = new AtomicInteger(1000);
    private static boolean loaded = false;

    static {
        loadLibrary();
    }

    private Vox3D() {
    }

    public static synchronized void loadLibrary() {
        if (loaded) {
            return;
        }

        final UnsatisfiedLinkError systemLoadFailure;
        try {
            // Try loading from java.library.path first
            System.loadLibrary(NATIVE_NAME);
            loaded = true;
            LOGGER.info("Successfully loaded native {} from java.library.path", NATIVE_NAME);
            return;
        } catch (final UnsatisfiedLinkError error) {
            systemLoadFailure = error;
        }

        // Extract from embedded resources
        final String osName = System.getProperty("os.name", "").toLowerCase();
        final String rawArch = System.getProperty("os.arch", "").toLowerCase();
        final String architecture = switch (rawArch) {
            case "amd64", "x86_64" -> "x86_64";
            case "aarch64", "arm64" -> "aarch64";
            default -> rawArch;
        };

        final String platform;
        final String fileName;
        final String extension;

        if (osName.contains("win")) {
            platform = "windows";
            fileName = "sable_vox3d.dll";
            extension = ".dll";
        } else if (osName.contains("linux")) {
            platform = "linux";
            fileName = "libsable_vox3d.so";
            extension = ".so";
        } else if (osName.contains("mac") || osName.contains("darwin")) {
            platform = "macos";
            fileName = "libsable_vox3d.dylib";
            extension = ".dylib";
        } else {
            final UnsatisfiedLinkError error = new UnsatisfiedLinkError(
                    "Vox3D does not provide a native library for operating system '" + osName + "'");
            error.addSuppressed(systemLoadFailure);
            throw error;
        }

        final String resourcePath = "/natives/sable_vox3d/" + platform + "/" + architecture + "/" + fileName;
        InputStream resource = Vox3D.class.getResourceAsStream(resourcePath);
        if (resource == null && platform.equals("windows") && architecture.equals("x86_64")) {
            resource = Vox3D.class.getResourceAsStream("/natives/sable_vox3d.dll");
        }

        if (resource == null) {
            final UnsatisfiedLinkError error = new UnsatisfiedLinkError(
                    "Vox3D native library is unavailable for " + platform + "/" + architecture
                            + " (expected resource " + resourcePath + ")");
            error.addSuppressed(systemLoadFailure);
            throw error;
        }

        try (final InputStream in = resource) {
            final File tempFile = Files.createTempFile("sable_vox3d_", extension).toFile();
            tempFile.deleteOnExit();

            try (final FileOutputStream out = new FileOutputStream(tempFile)) {
                final byte[] buffer = new byte[8192];
                int read;
                while ((read = in.read(buffer)) != -1) {
                    out.write(buffer, 0, read);
                }
            }

            System.load(tempFile.getAbsolutePath());
            loaded = true;
            LOGGER.info("Successfully extracted and loaded native {}", tempFile.getAbsolutePath());
        } catch (final Exception | UnsatisfiedLinkError e) {
            LOGGER.error("Failed to extract and load native library for Vox3D", e);
            final UnsatisfiedLinkError error = new UnsatisfiedLinkError(
                    "Could not load Vox3D native for " + platform + "/" + architecture + ": " + e.getMessage());
            error.initCause(e);
            error.addSuppressed(systemLoadFailure);
            throw error;
        }
    }

    public static int nextBodyID() {
        return BODY_ID_COUNTER.incrementAndGet();
    }

    public static int getID(@Nullable final PhysicsPipelineBody body) {
        return body == null ? -1 : body.getRuntimeId();
    }

    public static void setMassProperties(final long sceneHandle, final int id, final MassData massTracker) {
        final double mass = massTracker.getMass();
        final Vector3dc centerOfMass = massTracker.getCenterOfMass();
        final Matrix3dc inertia = massTracker.getInertiaTensor();

        final double[] comArray = centerOfMass != null
                ? new double[]{centerOfMass.x(), centerOfMass.y(), centerOfMass.z()}
                : new double[]{0.0, 0.0, 0.0};

        final double[] inertiaArray = inertia != null
                ? new double[]{
                inertia.m00(), inertia.m01(), inertia.m02(),
                inertia.m10(), inertia.m11(), inertia.m12(),
                inertia.m20(), inertia.m21(), inertia.m22()
        }
                : new double[]{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};

        setMassProperties(sceneHandle, id, mass, comArray, inertiaArray);
    }

    public static Vox3DVoxelColliderData createVoxelColliderEntry(final double frictionMultiplier, final double volume, final double restitution, final boolean isFluid, final BlockSubLevelCollisionCallback contactEvents) {
        return new Vox3DVoxelColliderData(
                newVoxelCollider(frictionMultiplier, volume, restitution, isFluid, contactEvents),
                isFluid,
                contactEvents
        );
    }

    // --- Native JNI Methods ---

    public static native long initialize(double gx, double gy, double gz, double universalDrag);
    public static native void dispose(long sceneHandle);
    public static native void tick(long sceneHandle, double timeStep);
    public static native void step(long sceneHandle, double timeStep);

    public static native void configFrequencyAndDamping(long sceneHandle, double frequency, double dampingRatio);
    public static native void configSolverIterations(long sceneHandle, int solverIterations, int pgsIterations, int stabilizationIterations);
    public static native void configMinIslandSize(long sceneHandle, int minDynamicBodiesPerIsland);

    public static native void createSubLevel(long sceneHandle, int id, double[] pose);
    public static native void removeSubLevel(long sceneHandle, int id);
    public static native void getPose(long sceneHandle, int id, double[] poseDest);
    public static native void setCenterOfMass(long sceneHandle, int id, double cx, double cy, double cz);
    public static native void setLocalBounds(long sceneHandle, int id, int minX, int minY, int minZ, int maxX, int maxY, int maxZ);
    public static native void setMassProperties(long sceneHandle, int id, double mass, double[] centerOfMass, double[] inertiaTensor);
    public static native void teleportObject(long sceneHandle, int id, double px, double py, double pz, double rx, double ry, double rz, double rw);
    public static native void applyForce(long sceneHandle, int id, double ox, double oy, double oz, double fx, double fy, double fz, boolean wakeUp);
    public static native void applyForceAndTorque(long sceneHandle, int id, double fx, double fy, double fz, double tx, double ty, double tz, boolean wakeUp);
    public static native void addLinearAngularVelocities(long sceneHandle, int id, double lx, double ly, double lz, double ax, double ay, double az, boolean wakeUp);
    public static native void getLinearVelocity(long sceneHandle, int id, double[] dest);
    public static native void getAngularVelocity(long sceneHandle, int id, double[] dest);
    public static native void wakeUpObject(long sceneHandle, int id);

    public static native void createKinematicContraption(long sceneHandle, int mountId, int id, double[] pose);
    public static native void removeKinematicContraption(long sceneHandle, int id);
    public static native void setKinematicContraptionTransform(long sceneHandle, int id, double[] centerOfMass, double[] pose, double[] velocity);
    public static native void addKinematicContraptionChunkSection(long sceneHandle, int id, int cx, int cy, int cz, int[] chunkData);

    public static native void addChunk(long sceneHandle, int cx, int cy, int cz, int[] chunkData, boolean global, int subLevelId);
    public static native void removeChunk(long sceneHandle, int cx, int cy, int cz, boolean global);
    public static native void changeBlock(long sceneHandle, int bx, int by, int bz, int packedValue);

    public static native int newVoxelCollider(double frictionMultiplier, double volume, double restitution, boolean isFluid, BlockSubLevelCollisionCallback contactEvents);
    public static native void addVoxelColliderBox(int index, double[] bounds);
    public static native void clearVoxelColliderBoxes(int index);

    public static native void createBox(long sceneHandle, int id, double mass, double hx, double hy, double hz, double[] pose);
    public static native void removeBox(long sceneHandle, int id);

    public static native long createRope(long sceneHandle, double radius, double segmentLength, double[] points, int pointCount);
    public static native void removeRope(long sceneHandle, long ropeHandle);
    public static native double[] queryRope(long sceneHandle, long ropeHandle);
    public static native void setRopeFirstSegmentLength(long sceneHandle, long ropeHandle, double length);
    public static native void addRopePointAtStart(long sceneHandle, long ropeHandle, double x, double y, double z);
    public static native void removeRopePointAtStart(long sceneHandle, long ropeHandle);
    public static native void setRopeAttachment(long sceneHandle, long ropeHandle, int subLevelId, double x, double y, double z, boolean isEnd);
    public static native void wakeUpRope(long sceneHandle, long ropeHandle);

    public static native long addFixedConstraint(long sceneHandle, int bodyA, int bodyB, double p1x, double p1y, double p1z, double p2x, double p2y, double p2z, double rx, double ry, double rz, double rw);
    public static native long addRotaryConstraint(long sceneHandle, int bodyA, int bodyB, double p1x, double p1y, double p1z, double p2x, double p2y, double p2z, double n1x, double n1y, double n1z, double n2x, double n2y, double n2z);
    public static native long addFreeConstraint(long sceneHandle, int bodyA, int bodyB, double p1x, double p1y, double p1z, double p2x, double p2y, double p2z, double rx, double ry, double rz, double rw);
    public static native long addGenericConstraint(long sceneHandle, int bodyA, int bodyB, double p1x, double p1y, double p1z, double r1x, double r1y, double r1z, double r1w, double p2x, double p2y, double p2z, double r2x, double r2y, double r2z, double r2w, int lockedAxesMask);

    public static native void setConstraintContactsEnabled(long sceneHandle, long constraintHandle, boolean enabled);
    public static native void getConstraintImpulses(long sceneHandle, long constraintHandle, double[] impulseDest);
    public static native void setConstraintMotor(long sceneHandle, long constraintHandle, int axisOrdinal, double target, double stiffness, double damping, boolean hasForceLimit, double maxForce);
    public static native void setConstraintFrame(long sceneHandle, long constraintHandle, int frameSide, double px, double py, double pz, double rx, double ry, double rz, double rw);
    public static native void setConstraintLimit(long sceneHandle, long constraintHandle, int axisOrdinal, double min, double max);
    public static native boolean lockConstraintAxes(long sceneHandle, long constraintHandle, byte mask);
    public static native void removeConstraint(long sceneHandle, long constraintHandle);
    public static native boolean isConstraintValid(long sceneHandle, long constraintHandle);

    public static native double[] clearCollisions(long sceneHandle);
    public static native void setWind(long sceneHandle, double wx, double wy, double wz, double drag, double lift);
}
