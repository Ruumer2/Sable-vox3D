package dev.ryanhcode.sable.physics.impl.vox3d;

import dev.ryanhcode.sable.Sable;
import dev.ryanhcode.sable.api.physics.PhysicsPipeline;
import dev.ryanhcode.sable.api.physics.PhysicsPipelineBody;
import dev.ryanhcode.sable.api.physics.constraint.*;
import dev.ryanhcode.sable.api.physics.mass.MassTracker;
import dev.ryanhcode.sable.api.physics.object.box.BoxHandle;
import dev.ryanhcode.sable.api.physics.object.box.BoxPhysicsObject;
import dev.ryanhcode.sable.api.physics.object.rope.RopeHandle;
import dev.ryanhcode.sable.api.physics.object.rope.RopePhysicsObject;
import dev.ryanhcode.sable.api.sublevel.KinematicContraption;
import dev.ryanhcode.sable.api.sublevel.ServerSubLevelContainer;
import dev.ryanhcode.sable.api.sublevel.SubLevelContainer;
import dev.ryanhcode.sable.companion.math.*;
import dev.ryanhcode.sable.physics.chunk.VoxelNeighborhoodState;
import dev.ryanhcode.sable.physics.config.PhysicsConfigData;
import dev.ryanhcode.sable.physics.impl.vox3d.box.Vox3DBoxHandle;
import dev.ryanhcode.sable.physics.impl.vox3d.collider.Vox3DVoxelColliderBakery;
import dev.ryanhcode.sable.physics.impl.vox3d.collider.Vox3DVoxelColliderData;
import dev.ryanhcode.sable.physics.impl.vox3d.constraint.fixed.Vox3DFixedConstraintHandle;
import dev.ryanhcode.sable.physics.impl.vox3d.constraint.free.Vox3DFreeConstraintHandle;
import dev.ryanhcode.sable.physics.impl.vox3d.constraint.generic.Vox3DGenericConstraintHandle;
import dev.ryanhcode.sable.physics.impl.vox3d.constraint.rotary.Vox3DRotaryConstraintHandle;
import dev.ryanhcode.sable.physics.impl.vox3d.rope.Vox3DRopeHandle;
import dev.ryanhcode.sable.sublevel.ServerSubLevel;
import dev.ryanhcode.sable.sublevel.SubLevel;
import dev.ryanhcode.sable.sublevel.plot.LevelPlot;
import dev.ryanhcode.sable.sublevel.system.SubLevelPhysicsSystem;
import dev.ryanhcode.sable.util.LevelAccelerator;
import dev.ryanhcode.sable.util.SableMathUtils;
import it.unimi.dsi.fastutil.ints.Int2ObjectArrayMap;
import it.unimi.dsi.fastutil.ints.Int2ObjectMap;
import it.unimi.dsi.fastutil.longs.Long2LongOpenHashMap;
import it.unimi.dsi.fastutil.longs.Long2ObjectMap;
import it.unimi.dsi.fastutil.longs.Long2ObjectOpenHashMap;
import it.unimi.dsi.fastutil.objects.Object2ObjectMap;
import it.unimi.dsi.fastutil.objects.Object2ObjectOpenHashMap;
import it.unimi.dsi.fastutil.objects.ReferenceArrayList;
import it.unimi.dsi.fastutil.objects.ReferenceList;
import net.minecraft.CrashReport;
import net.minecraft.CrashReportCategory;
import net.minecraft.ReportedException;
import net.minecraft.core.BlockPos;
import net.minecraft.core.Direction;
import net.minecraft.core.SectionPos;
import net.minecraft.core.particles.BlockParticleOption;
import net.minecraft.core.particles.ParticleTypes;
import net.minecraft.server.level.ServerLevel;
import net.minecraft.sounds.SoundSource;
import net.minecraft.world.level.block.Blocks;
import net.minecraft.world.level.block.SoundType;
import net.minecraft.world.level.block.state.BlockState;
import net.minecraft.world.level.chunk.LevelChunk;
import net.minecraft.world.level.chunk.LevelChunkSection;
import net.minecraft.world.phys.Vec3;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import org.joml.Quaterniond;
import org.joml.Quaterniondc;
import org.joml.Vector3d;
import org.joml.Vector3dc;

/**
 * Implementation of {@link PhysicsPipeline} for the Vox3D / Box3D physics engine.
 */
public class Vox3DPhysicsPipeline implements PhysicsPipeline {

    private static final double DISTANCE_THRESHOLD = 1e-7;
    private static final double ANGULAR_THRESHOLD = 1e-7;

    private final ServerLevel level;
    private final LevelAccelerator accelerator;
    private final Vox3DVoxelColliderBakery colliderBakery;
    private final Int2ObjectMap<ServerSubLevel> activeSubLevels = new Int2ObjectArrayMap<>();
    private final Object2ObjectMap<KinematicContraption, TrackedKinematicContraption> activeContraptions = new Object2ObjectOpenHashMap<>();
    private final Long2LongOpenHashMap recentCollisions = new Long2LongOpenHashMap();
    private final ReferenceList<PhysicsPipelineBody> queuedWakeUps = new ReferenceArrayList<>();
    private final double[] poseCache;
    private Vox3DPhysicsScene scene;

    public Vox3DPhysicsPipeline(final ServerLevel level) {
        this.level = level;
        this.accelerator = new LevelAccelerator(level);
        this.colliderBakery = new Vox3DVoxelColliderBakery(this.accelerator);
        this.recentCollisions.defaultReturnValue(-1);
        this.poseCache = new double[7];
    }

    private static int packBlockState(final VoxelNeighborhoodState state, final int colliderID) {
        return ((int) state.byteRepresentation()) | (colliderID << 16);
    }

    public long getSceneHandle() {
        if (this.scene == null) {
            throw new IllegalStateException("Vox3D Physics scene is not initialized");
        }
        return this.scene.handle();
    }

    @Override
    public void init(@Nullable final Vector3dc gravity, final double universalDrag) {
        try {
            final double gx = gravity != null ? gravity.x() : 0.0;
            final double gy = gravity != null ? gravity.y() : -9.81;
            final double gz = gravity != null ? gravity.z() : 0.0;
            this.scene = new Vox3DPhysicsScene(Vox3D.initialize(gx, gy, gz, universalDrag));
            Vox3D.setWind(this.scene.handle(), 0.0, 0.0, 0.0, 1.0, 1.0);
            Sable.LOGGER.info("Initialized Sable Vox3D physics scene with handle: {}", this.scene.handle());
        } catch (final LinkageError e) {
            Sable.LOGGER.error("Sable has failed to link with native Vox3D physics library", e);
            final Throwable cause = e.getCause() != null ? e.getCause() : e;
            final CrashReport crashReport = CrashReport.forThrowable(cause, "Sable linking with Vox3D natives");
            final CrashReportCategory category = crashReport.addCategory("Natives");
            category.setDetail("Name", Vox3D.NATIVE_NAME);
            throw new ReportedException(crashReport);
        }
    }

    @Override
    public void dispose() {
        if (this.scene != null) {
            Vox3D.dispose(this.scene.handle());
            this.scene = null;
        }
        this.activeSubLevels.clear();
        this.activeContraptions.clear();
        this.recentCollisions.clear();
        this.queuedWakeUps.clear();
    }

    @Override
    public void prePhysicsTicks() {
        final double timeStep = 1.0 / 20.0;
        Vox3D.tick(this.scene.handle(), timeStep);
    }

    @Override
    public void physicsTick(final double timeStep) {
        this.updateContraptionPoses();
        Vox3D.step(this.scene.handle(), timeStep);

        for (final PhysicsPipelineBody queuedWakeUp : this.queuedWakeUps) {
            if (queuedWakeUp.isRemoved()) {
                continue;
            }
            Vox3D.wakeUpObject(this.scene.handle(), Vox3D.getID(queuedWakeUp));
        }

        this.queuedWakeUps.clear();
    }

    @Override
    public void postPhysicsTicks() {
        this.processCollisionEffects();
    }

    @Override
    public void tick() {
        this.accelerator.clearCache();
    }

    @Override
    public void add(final ServerSubLevel subLevel, final Pose3dc pose) {
        this.assertBodyValid(subLevel);
        final Vector3dc pos = pose.position();
        final Quaterniondc rot = pose.orientation();

        final int id = Vox3D.getID(subLevel);
        if (this.activeSubLevels.containsKey(id)) {
            throw new IllegalStateException("Sub-level runtime ID " + id + " is already present in the pipeline");
        }
        Vox3D.createSubLevel(this.scene.handle(), id, new double[]{pos.x(), pos.y(), pos.z(), rot.x(), rot.y(), rot.z(), rot.w()});

        subLevel.updateMergedMassData(1.0f);
        final Vector3dc centerOfMass = subLevel.getMassTracker().getCenterOfMass();

        if (centerOfMass != null) {
            subLevel.logicalPose().rotationPoint().set(centerOfMass);
        }
        this.onStatsChanged(subLevel);

        this.activeSubLevels.put(id, subLevel);
    }

    @Override
    public void remove(final ServerSubLevel subLevel) {
        final int id = Vox3D.getID(subLevel);
        Vox3D.removeSubLevel(this.scene.handle(), id);
        this.activeSubLevels.remove(id);
    }

    @Override
    public void add(final KinematicContraption contraption) {
        if (this.activeContraptions.containsKey(contraption)) {
            throw new IllegalStateException("Contraption " + contraption + " is already present in pipeline");
        }

        final int id = this.getNextRuntimeID();
        final SubLevel mountSubLevel = Sable.HELPER.getContaining(this.level, contraption.sable$getPosition());
        final int mountId = mountSubLevel != null ? Vox3D.getID((ServerSubLevel) mountSubLevel) : -1;

        final Vector3dc mountCenterOfMass = mountSubLevel != null
                ? ((ServerSubLevel) mountSubLevel).getMassTracker().getCenterOfMass()
                : null;
        final Vector3dc parentCenterOfMass = mountCenterOfMass != null ? mountCenterOfMass : JOMLConversion.ZERO;

        // The native create call does not receive the contraption COM. Use an
        // infinite position sentinel so the first pose update is never skipped,
        // including for an identity-oriented contraption placed at the origin.
        this.activeContraptions.put(contraption, new TrackedKinematicContraption(
                new Vector3d(Double.POSITIVE_INFINITY, 0.0, 0.0), new Quaterniond(),
                new Vector3d(), new Vector3d(), id, mountId));

        final BoundingBox3i localBounds = new BoundingBox3i();
        contraption.sable$getLocalBounds(localBounds);

        final Vector3d pos = new Vector3d(contraption.sable$getPosition());
        if (mountSubLevel != null) {
            pos.sub(parentCenterOfMass);
        }
        final Quaterniond rot = contraption.sable$getOrientation();
        final double[] pose = {pos.x(), pos.y(), pos.z(), rot.x(), rot.y(), rot.z(), rot.w()};

        Vox3D.createKinematicContraption(this.scene.handle(), mountId, id, pose);

        record UploadingContraptionChunk(int[] data) {}
        final Long2ObjectMap<UploadingContraptionChunk> chunks = new Long2ObjectOpenHashMap<>();

        final BlockPos.MutableBlockPos blockPos = new BlockPos.MutableBlockPos();
        for (int x = localBounds.minX(); x <= localBounds.maxX(); x++) {
            for (int z = localBounds.minZ(); z <= localBounds.maxZ(); z++) {
                for (int y = localBounds.minY(); y <= localBounds.maxY(); y++) {
                    final BlockState blockState = contraption.sable$blockGetter().getBlockState(blockPos.set(x, y, z));

                    if (blockState.isAir()) continue;

                    final SectionPos sectionPos = SectionPos.of(blockPos);
                    final UploadingContraptionChunk chunk = chunks.computeIfAbsent(sectionPos.asLong(), longPos -> new UploadingContraptionChunk(new int[LevelChunkSection.SECTION_SIZE]));

                    final VoxelNeighborhoodState state = VoxelNeighborhoodState.CORNER;
                    final Vox3DVoxelColliderData colliderData = this.colliderBakery.getPhysicsDataForBlock(blockState);

                    final int index = (x & 15) + ((z & 15) << 4) + ((y & 15) << 8);
                    final int colliderValue = colliderData == null ? 0 : colliderData.handle() + 1;
                    chunk.data[index] = packBlockState(state, colliderValue);
                }
            }
        }

        if (contraption.sable$shouldCollide()) {
            for (final Long2ObjectMap.Entry<UploadingContraptionChunk> entry : chunks.long2ObjectEntrySet()) {
                final SectionPos sectionPos = SectionPos.of(entry.getLongKey());
                final UploadingContraptionChunk chunk = entry.getValue();
                Vox3D.addKinematicContraptionChunkSection(this.scene.handle(), id, sectionPos.x(), sectionPos.y(), sectionPos.z(), chunk.data());
            }
        }

        this.updateContraptionPose(contraption, 1.0f);
        Vox3D.setLocalBounds(this.scene.handle(), id, localBounds.minX, localBounds.minY, localBounds.minZ, localBounds.maxX, localBounds.maxY, localBounds.maxZ);
    }

    @Override
    public void remove(final KinematicContraption contraption) {
        final TrackedKinematicContraption removed = this.activeContraptions.remove(contraption);
        if (removed == null) {
            return;
        }
        Vox3D.removeKinematicContraption(this.scene.handle(), removed.id());
    }

    @Override
    public Pose3d readPose(final ServerSubLevel subLevel, final Pose3d dest) {
        this.assertBodyValid(subLevel);
        Vox3D.getPose(this.scene.handle(), Vox3D.getID(subLevel), this.poseCache);

        dest.position().set(this.poseCache[0], this.poseCache[1], this.poseCache[2]);
        dest.orientation().set(this.poseCache[3], this.poseCache[4], this.poseCache[5], this.poseCache[6]);

        return dest;
    }

    @Override
    public RopeHandle addRope(final RopePhysicsObject rope) {
        return Vox3DRopeHandle.create(this.scene.handle(), rope.getCollisionRadius(), rope.getPoints());
    }

    @Override
    public BoxHandle addBox(final BoxPhysicsObject box) {
        return Vox3DBoxHandle.create(this.scene.handle(), box.getPose(), box.getHalfExtents(), box.getMass());
    }

    @Override
    public void handleChunkSectionAddition(final LevelChunkSection section, final int x, final int y, final int z, final boolean uploadDataIfGlobal) {
        this.accelerator.clearCache();

        final int[] array = new int[LevelChunkSection.SECTION_SIZE];
        final SectionPos sectionPos = SectionPos.of(x, y, z);

        if (!section.hasOnlyAir()) {
            final LevelChunk chunk = this.accelerator.getChunk(x, z);

            for (int bx = 0; bx < 16; bx++) {
                for (int bz = 0; bz < 16; bz++) {
                    for (int by = 0; by < 16; by++) {
                        final BlockPos globalPos = new BlockPos(bx, by, bz).offset(sectionPos.minBlockX(), sectionPos.minBlockY(), sectionPos.minBlockZ());
                        final VoxelNeighborhoodState state = VoxelNeighborhoodState.getState(this.accelerator, globalPos, chunk);
                        final Vox3DVoxelColliderData colliderData = this.colliderBakery.getPhysicsDataForBlock(this.accelerator.getBlockState(globalPos));

                        final int index = bx + (bz << 4) + (by << 8);
                        final int colliderValue = colliderData == null ? 0 : colliderData.handle() + 1;
                        array[index] = packBlockState(state, colliderValue);
                    }
                }
            }
        }

        final LevelPlot plot = SubLevelContainer.getContainer(this.level).getPlot(x, z);
        final boolean global = plot == null;
        int id = -1;

        if (plot != null && uploadDataIfGlobal) {
            id = Vox3D.getID((ServerSubLevel) plot.getSubLevel());
        }
        Vox3D.addChunk(this.scene.handle(), x, y, z, array, global, id);
    }

    @Override
    public void handleChunkSectionRemoval(final int x, final int y, final int z) {
        Vox3D.removeChunk(this.scene.handle(), x, y, z, !SubLevelContainer.getContainer(this.level).inBounds(x, z));
    }

    @Override
    public void handleBlockChange(final SectionPos sectionPos, final LevelChunkSection chunk, int x, int y, int z, final BlockState oldState, final BlockState newState) {
        this.accelerator.clearCache();

        x = (sectionPos.x() << 4) + x;
        y = (sectionPos.y() << 4) + y;
        z = (sectionPos.z() << 4) + z;

        final BlockPos globalBlockPos = new BlockPos(x, y, z);

        for (final Direction dir : Direction.values()) {
            final BlockPos pos = globalBlockPos.relative(dir);
            final VoxelNeighborhoodState state = VoxelNeighborhoodState.getState(this.accelerator, pos, null);
            final Vox3DVoxelColliderData colliderData = this.colliderBakery.getPhysicsDataForBlock(this.level.getBlockState(pos));

            final int colliderValue = colliderData == null ? 0 : colliderData.handle() + 1;
            Vox3D.changeBlock(this.scene.handle(), pos.getX(), pos.getY(), pos.getZ(), packBlockState(state, colliderValue));
        }

        final VoxelNeighborhoodState state = VoxelNeighborhoodState.getState(this.accelerator, globalBlockPos, null);
        final Vox3DVoxelColliderData colliderData = this.colliderBakery.getPhysicsDataForBlock(newState);

        final int colliderValue = colliderData == null ? 0 : colliderData.handle() + 1;
        Vox3D.changeBlock(this.scene.handle(), x, y, z, packBlockState(state, colliderValue));
    }

    @Override
    public void onStatsChanged(@NotNull final ServerSubLevel subLevel) {
        this.assertBodyValid(subLevel);

        final BoundingBox3ic plotBounds = subLevel.getPlot().getBoundingBox();
        final int id = Vox3D.getID(subLevel);

        Vox3D.setLocalBounds(this.scene.handle(), id, plotBounds.minX(), plotBounds.minY(), plotBounds.minZ(), plotBounds.maxX(), plotBounds.maxY(), plotBounds.maxZ());

        final Vector3dc centerOfMass = subLevel.getMassTracker().getCenterOfMass();
        if (centerOfMass != null) {
            Vox3D.setCenterOfMass(this.scene.handle(), id, centerOfMass.x(), centerOfMass.y(), centerOfMass.z());
            Vox3D.setMassProperties(this.scene.handle(), id, subLevel.getMassTracker());
        }
    }

    @Override
    public void teleport(final PhysicsPipelineBody body, final Vector3dc position, final Quaterniondc orientation) {
        this.assertBodyValid(body);
        Vox3D.teleportObject(this.scene.handle(), Vox3D.getID(body), position.x(), position.y(), position.z(), orientation.x(), orientation.y(), orientation.z(), orientation.w());
        if (body instanceof final ServerSubLevel subLevel) {
            subLevel.logicalPose().position().set(position);
            subLevel.logicalPose().orientation().set(orientation);
        }
    }

    @Override
    public void applyImpulse(final PhysicsPipelineBody body, final Vector3dc position, final Vector3dc force) {
        this.assertBodyValid(body);
        final Vector3dc centerOfMass = body.getMassTracker().getCenterOfMass();
        final double comX = centerOfMass != null ? centerOfMass.x() : 0.0;
        final double comY = centerOfMass != null ? centerOfMass.y() : 0.0;
        final double comZ = centerOfMass != null ? centerOfMass.z() : 0.0;
        Vox3D.applyForce(this.scene.handle(), Vox3D.getID(body), position.x() - comX, position.y() - comY, position.z() - comZ, force.x(), force.y(), force.z(), true);
    }

    @Override
    public void applyLinearAndAngularImpulse(final PhysicsPipelineBody body, final Vector3dc force, final Vector3dc torque, final boolean wakeUp) {
        this.assertBodyValid(body);
        Vox3D.applyForceAndTorque(this.scene.handle(), Vox3D.getID(body), force.x(), force.y(), force.z(), torque.x(), torque.y(), torque.z(), wakeUp);
    }

    @Override
    public void addLinearAndAngularVelocity(final PhysicsPipelineBody body, final Vector3dc linearVelocity, final Vector3dc angularVelocity) {
        this.assertBodyValid(body);
        Vox3D.addLinearAngularVelocities(this.scene.handle(), Vox3D.getID(body), linearVelocity.x(), linearVelocity.y(), linearVelocity.z(), angularVelocity.x(), angularVelocity.y(), angularVelocity.z(), true);
    }

    @Override
    public Vector3d getLinearVelocity(final PhysicsPipelineBody body, final Vector3d dest) {
        this.assertBodyValid(body);
        Vox3D.getLinearVelocity(this.scene.handle(), Vox3D.getID(body), this.poseCache);
        return dest.set(this.poseCache[0], this.poseCache[1], this.poseCache[2]);
    }

    @Override
    public Vector3d getAngularVelocity(final PhysicsPipelineBody body, final Vector3d dest) {
        this.assertBodyValid(body);
        Vox3D.getAngularVelocity(this.scene.handle(), Vox3D.getID(body), this.poseCache);
        return dest.set(this.poseCache[0], this.poseCache[1], this.poseCache[2]);
    }

    @Override
    public void wakeUp(final PhysicsPipelineBody body) {
        this.assertBodyValid(body);
        if (!SubLevelPhysicsSystem.IN_PHYSICS_STEP) {
            Vox3D.wakeUpObject(this.scene.handle(), Vox3D.getID(body));
        } else {
            this.queuedWakeUps.add(body);
        }
    }

    @SuppressWarnings("unchecked")
    @Override
    @Nullable
    public <T extends PhysicsConstraintHandle> T addConstraint(@Nullable final PhysicsPipelineBody bodyA, @Nullable final PhysicsPipelineBody bodyB, @NotNull final PhysicsConstraintConfiguration<T> configuration) {
        if (bodyA == null && bodyB == null) {
            throw new IllegalArgumentException("Cannot add a constraint between the static world and static world");
        }

        if (bodyA == bodyB) {
            throw new IllegalArgumentException("Cannot add a constraint between a body and itself");
        }

        try {
            configuration.validate(ServerSubLevelContainer.getContainer(this.level), bodyA, bodyB);
        } catch (final Exception e) {
            throw new IllegalArgumentException("Constraint validation failed", e);
        }

        final T constraint = switch (configuration) {
            case final RotaryConstraintConfiguration config ->
                    (T) Vox3DRotaryConstraintHandle.create(this.scene.handle(), bodyA, bodyB, config);
            case final FixedConstraintConfiguration config ->
                    (T) Vox3DFixedConstraintHandle.create(this.scene.handle(), bodyA, bodyB, config);
            case final FreeConstraintConfiguration config ->
                    (T) Vox3DFreeConstraintHandle.create(this.scene.handle(), bodyA, bodyB, config);
            case final GenericConstraintConfiguration config ->
                    (T) Vox3DGenericConstraintHandle.create(this.scene.handle(), bodyA, bodyB, config);
        };

        if (!constraint.isValid()) {
            return null;
        }

        return constraint;
    }

    @Override
    public void updateConfigFrom(final PhysicsConfigData data) {
        final long sceneHandle = this.getSceneHandle();
        Vox3D.configFrequencyAndDamping(sceneHandle, data.contactSpringFrequency, data.contactSpringDampingRatio);
        Vox3D.configSolverIterations(sceneHandle, data.solverIterations, data.pgsIterations, data.stabilizationIterations);
        Vox3D.configMinIslandSize(sceneHandle, data.minDynamicBodiesPerIsland);
    }

    @Override
    public int getNextRuntimeID() {
        return Vox3D.nextBodyID();
    }

    private void assertBodyValid(final PhysicsPipelineBody body) {
        if (body.isRemoved()) {
            throw new RuntimeException("Body has been removed");
        }
    }

    private void updateContraptionPoses() {
        final SubLevelPhysicsSystem system = SubLevelPhysicsSystem.require(this.level);
        final double partialPhysicsTick = system.getPartialPhysicsTick();

        for (final KinematicContraption contraption : this.activeContraptions.keySet()) {
            this.updateContraptionPose(contraption, partialPhysicsTick);
        }
    }

    private void updateContraptionPose(final KinematicContraption contraption, final double partialPhysicsTick) {
        final TrackedKinematicContraption trackedContraption = this.activeContraptions.get(contraption);

        final SubLevel mountSubLevel = Sable.HELPER.getContaining(this.level, contraption.sable$getPosition());
        final int currentMountId = mountSubLevel != null ? Vox3D.getID((ServerSubLevel) mountSubLevel) : -1;
        if (currentMountId != trackedContraption.mountId()) {
            trackedContraption.setMountId(currentMountId);
            Vox3D.setKinematicContraptionMount(this.scene.handle(), trackedContraption.id(), currentMountId);
        }

        final Vector3dc mountCenterOfMass = mountSubLevel != null
                ? ((ServerSubLevel) mountSubLevel).getMassTracker().getCenterOfMass()
                : null;
        final Vector3dc parentCenterOfMass = mountCenterOfMass != null ? mountCenterOfMass : JOMLConversion.ZERO;

        final Vector3dc lastPosition = new Vector3d(contraption.sable$getPosition(partialPhysicsTick - 1.0f));
        final Quaterniondc lastOrientation = new Quaterniond(contraption.sable$getOrientation(partialPhysicsTick - 1.0f));

        final Vector3d pos = new Vector3d(contraption.sable$getPosition(partialPhysicsTick));
        final Quaterniondc rot = contraption.sable$getOrientation(partialPhysicsTick);

        final Vector3d linVel = pos.sub(lastPosition, new Vector3d());
        final Vector3d angVel = SableMathUtils.getAngularVelocity(lastOrientation, rot, new Vector3d());

        linVel.mul(20.0);
        angVel.mul(20.0);
        rot.transformInverse(linVel);
        rot.transformInverse(angVel);

        if (linVel.lengthSquared() > 400.0) {
            linVel.zero();
        }
        if (angVel.lengthSquared() > 400.0) {
            angVel.zero();
        }

        pos.sub(parentCenterOfMass);

        if (
                pos.distanceSquared(trackedContraption.lastUploadedPosition()) > DISTANCE_THRESHOLD * DISTANCE_THRESHOLD ||
                        linVel.distanceSquared(trackedContraption.lastUploadedLinVel()) > DISTANCE_THRESHOLD * DISTANCE_THRESHOLD ||
                        angVel.distanceSquared(trackedContraption.lastUploadedAngVel()) > DISTANCE_THRESHOLD * DISTANCE_THRESHOLD ||
                        rot.div(trackedContraption.lastUploadedOrientation(), new Quaterniond()).angle() > ANGULAR_THRESHOLD
        ) {
            final MassTracker massTracker = contraption.sable$getMassTracker();
            final Vector3dc centerOfMass = massTracker != null ? massTracker.getCenterOfMass() : null;

            final double[] centerOfMassArray = new double[]{centerOfMass != null ? centerOfMass.x() : 0.0, centerOfMass != null ? centerOfMass.y() : 0.0, centerOfMass != null ? centerOfMass.z() : 0.0};
            final double[] poseArray = {pos.x(), pos.y(), pos.z(), rot.x(), rot.y(), rot.z(), rot.w()};
            final double[] velocityArray = {linVel.x(), linVel.y(), linVel.z(), angVel.x(), angVel.y(), angVel.z()};
            Vox3D.setKinematicContraptionTransform(this.scene.handle(), trackedContraption.id(), centerOfMassArray, poseArray, velocityArray);

            trackedContraption.lastUploadedPosition().set(pos);
            trackedContraption.lastUploadedLinVel().set(linVel);
            trackedContraption.lastUploadedAngVel().set(angVel);
            trackedContraption.lastUploadedOrientation().set(rot);
        }
    }

    private void processCollisionEffects() {
        this.recentCollisions.long2LongEntrySet().removeIf(entry -> this.level.getGameTime() - entry.getLongValue() > 2);

        final Vector3d localPointA = new Vector3d();
        final Vector3d localPointB = new Vector3d();
        final Vector3d localNormalA = new Vector3d();
        final Vector3d localNormalB = new Vector3d();

        final Vector3d globalPointA = new Vector3d();
        final Vector3d globalPointB = new Vector3d();

        final double[] collisions = Vox3D.clearCollisions(this.scene.handle());

        final BlockPos.MutableBlockPos pos = new BlockPos.MutableBlockPos();
        final BlockPos.MutableBlockPos cornerPos = new BlockPos.MutableBlockPos();

        for (int i = 0; i < collisions.length / 15; i++) {
            final int startIndex = i * 15;
            final int idA = (int) collisions[startIndex];
            final int idB = (int) collisions[startIndex + 1];

            final double forceAmount = collisions[startIndex + 2];
            localNormalA.set(collisions[startIndex + 3], collisions[startIndex + 4], collisions[startIndex + 5]);
            localNormalB.set(collisions[startIndex + 6], collisions[startIndex + 7], collisions[startIndex + 8]);
            localPointA.set(collisions[startIndex + 9], collisions[startIndex + 10], collisions[startIndex + 11]);
            localPointB.set(collisions[startIndex + 12], collisions[startIndex + 13], collisions[startIndex + 14]);

            final ServerSubLevel subLevelA = this.activeSubLevels.get(idA);
            final ServerSubLevel subLevelB = this.activeSubLevels.get(idB);

            final double minMass = Math.min(subLevelA != null ? subLevelA.getMassTracker().getMass() : Double.MAX_VALUE, subLevelB != null ? subLevelB.getMassTracker().getMass() : Double.MAX_VALUE);

            if (forceAmount > 25.0 * minMass) {
                BlockState stateA = Blocks.STONE.defaultBlockState();
                BlockState stateB = stateA;

                if (subLevelA != null) {
                    final Pose3d p = subLevelA.logicalPose();
                    pos.set(localPointA.x + p.rotationPoint().x, localPointA.y + p.rotationPoint().y, localPointA.z + p.rotationPoint().z);
                    cornerPos.set(localPointA.x + p.rotationPoint().x + 0.5, localPointA.y + p.rotationPoint().y + 0.5, localPointA.z + p.rotationPoint().z + 0.5);

                    final long exists = this.recentCollisions.put(cornerPos.asLong(), this.level.getGameTime());
                    if (exists != -1) continue;
                    stateA = this.accelerator.getBlockState(pos);
                }

                if (subLevelB != null) {
                    final Pose3d p = subLevelB.logicalPose();
                    pos.set(localPointB.x + p.rotationPoint().x, localPointB.y + p.rotationPoint().y, localPointB.z + p.rotationPoint().z);
                    cornerPos.set(localPointB.x + p.rotationPoint().x + 0.5, localPointB.y + p.rotationPoint().y + 0.5, localPointB.z + p.rotationPoint().z + 0.5);

                    final long exists = this.recentCollisions.put(cornerPos.asLong(), this.level.getGameTime());
                    if (exists != -1) continue;
                    stateB = this.accelerator.getBlockState(pos);
                }

                globalPointA.set(localPointA);
                globalPointB.set(localPointB);

                if (subLevelA != null) {
                    final Pose3d p = subLevelA.logicalPose();
                    p.orientation().transform(globalPointA).add(p.position());
                }

                if (subLevelB != null) {
                    final Pose3d p = subLevelB.logicalPose();
                    p.orientation().transform(globalPointB).add(p.position());
                }

                final BlockState state = stateB;
                this.level.sendParticles(new BlockParticleOption(ParticleTypes.BLOCK, state), globalPointA.x, globalPointA.y, globalPointA.z, 2, 0.0, 0.0, 0.0, 0.1);

                final Vec3 position = JOMLConversion.toMojang(globalPointA);
                final float volumeScale = 0.4f;
                final SoundType soundType = state.getSoundType();

                this.level.playSound(null, position.x, position.y, position.z, soundType.getStepSound(), SoundSource.BLOCKS, 0.2f * volumeScale, (float) (0.6 - 0.2 + Math.random() * 0.4));
                this.level.playSound(null, position.x, position.y, position.z, soundType.getHitSound(), SoundSource.BLOCKS, 0.2f * volumeScale, (float) (Math.random() * 0.4));
                this.level.playSound(null, position.x, position.y, position.z, soundType.getPlaceSound(), SoundSource.BLOCKS, 0.2f * volumeScale, (float) (0.5 - 0.2 + Math.random() * 0.4));
            }
        }
    }

    private static final class TrackedKinematicContraption {
        private final Vector3d lastUploadedPosition;
        private final Quaterniond lastUploadedOrientation;
        private final Vector3d lastUploadedLinVel;
        private final Vector3d lastUploadedAngVel;
        private final int id;
        private int mountId;

        TrackedKinematicContraption(final Vector3d lastUploadedPosition, final Quaterniond lastUploadedOrientation,
                                    final Vector3d lastUploadedLinVel, final Vector3d lastUploadedAngVel,
                                    final int id, final int mountId) {
            this.lastUploadedPosition = lastUploadedPosition;
            this.lastUploadedOrientation = lastUploadedOrientation;
            this.lastUploadedLinVel = lastUploadedLinVel;
            this.lastUploadedAngVel = lastUploadedAngVel;
            this.id = id;
            this.mountId = mountId;
        }

        public Vector3d lastUploadedPosition() { return this.lastUploadedPosition; }
        public Quaterniond lastUploadedOrientation() { return this.lastUploadedOrientation; }
        public Vector3d lastUploadedLinVel() { return this.lastUploadedLinVel; }
        public Vector3d lastUploadedAngVel() { return this.lastUploadedAngVel; }
        public int id() { return this.id; }
        public int mountId() { return this.mountId; }
        public void setMountId(final int mountId) { this.mountId = mountId; }
    }
}
