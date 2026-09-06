package dev.ryanhcode.sable.physics.impl.vox3d.collider;

import dev.ryanhcode.sable.api.block.BlockSubLevelCollisionShape;
import dev.ryanhcode.sable.api.block.BlockSubLevelLiftProvider;
import dev.ryanhcode.sable.api.block.BlockWithSubLevelCollisionCallback;
import dev.ryanhcode.sable.api.physics.callback.BlockSubLevelCollisionCallback;
import dev.ryanhcode.sable.api.physics.collider.SableCollisionContext;
import dev.ryanhcode.sable.companion.math.JOMLConversion;
import dev.ryanhcode.sable.physics.chunk.VoxelNeighborhoodState;
import dev.ryanhcode.sable.physics.config.block_properties.PhysicsBlockPropertyHelper;
import dev.ryanhcode.sable.physics.impl.vox3d.Vox3D;
import net.minecraft.Util;
import net.minecraft.core.BlockPos;
import net.minecraft.core.Direction;
import net.minecraft.world.level.BlockGetter;
import net.minecraft.world.level.block.Blocks;
import net.minecraft.world.level.block.state.BlockState;
import net.minecraft.world.level.block.state.properties.BlockStateProperties;
import net.minecraft.world.phys.shapes.VoxelShape;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import org.joml.Vector3d;

import java.util.Objects;
import java.util.function.Function;

/**
 * A collider bakery that creates and caches collision shapes for blocks in Vox3D
 */
public class Vox3DVoxelColliderBakery {
    private final @NotNull PhysicsColliderBlockGetter level;
    private final Function<BlockState, Vox3DVoxelColliderData> blockPhysicsDataBuilder = Util.memoize(this::buildPhysicsDataForBlock);

    public Vox3DVoxelColliderBakery(@NotNull final BlockGetter blockGetter) {
        this.level = new PhysicsColliderBlockGetter(blockGetter);
    }

    public @NotNull BlockGetter getLevel() {
        return this.level;
    }

    private @NotNull Vox3DVoxelColliderData buildPhysicsDataForBlock(final BlockState childState) {
        final boolean liquid = VoxelNeighborhoodState.isLiquid(childState);

        final double friction = PhysicsBlockPropertyHelper.getFriction(childState);
        final double volume = PhysicsBlockPropertyHelper.getVolume(childState);
        final double restitution = PhysicsBlockPropertyHelper.getRestitution(childState);
        final BlockSubLevelCollisionCallback callback = BlockWithSubLevelCollisionCallback.sable$getCallback(childState);
        final Vox3DVoxelColliderData entry = Vox3D.createVoxelColliderEntry(friction, volume, restitution, liquid, callback);

        if (liquid) {
            entry.addBox(JOMLConversion.ZERO, new Vector3d(1.0, 1.0, 1.0));
            return entry;
        }

        final VoxelShape shape;

        this.level.setup(childState);
        if (childState.getBlock() instanceof final BlockSubLevelCollisionShape extension) {
            shape = extension.getSubLevelCollisionShape(this.level, childState);
        } else {
            shape = childState.getCollisionShape(this.level, BlockPos.ZERO, SableCollisionContext.get());
        }
        this.level.setup(Blocks.AIR.defaultBlockState());

        if (shape.isEmpty()) {
            return Vox3DVoxelColliderData.EMPTY;
        }

        shape.forAllBoxes((minX, minY, minZ, maxX, maxY, maxZ) -> {
            entry.addBox(
                    new Vector3d(Math.max(minX, 0.0), Math.max(minY, 0.0), Math.max(minZ, 0.0)),
                    new Vector3d(Math.min(maxX, 1.0), Math.min(maxY, 1.0), Math.min(maxZ, 1.0))
            );
        });

        if (isSail(childState)) {
            final Direction normal = getSailNormal(childState);
            final double nx = normal.getStepX();
            final double ny = normal.getStepY();
            final double nz = normal.getStepZ();

            // Derive chord direction if explicitly oriented perpendicular to the normal
            double cx = 0.0;
            double cy = 0.0;
            double cz = 0.0;

            if (childState.hasProperty(BlockStateProperties.HORIZONTAL_FACING)) {
                final Direction facing = childState.getValue(BlockStateProperties.HORIZONTAL_FACING);
                if (facing != normal && facing != normal.getOpposite()) {
                    cx = facing.getStepX();
                    cy = facing.getStepY();
                    cz = facing.getStepZ();
                }
            } else if (childState.hasProperty(BlockStateProperties.FACING)) {
                final Direction facing = childState.getValue(BlockStateProperties.FACING);
                if (facing != normal && facing != normal.getOpposite()) {
                    cx = facing.getStepX();
                    cy = facing.getStepY();
                    cz = facing.getStepZ();
                }
            }

            // Normal sails use symmetric airfoil profile (flat membrane canvas).
            // If chord is (0, 0, 0), Box3D's adaptive in-plane chord automatically aligns
            // with oncoming airflow across all 360 degrees of flight.
            entry.setAirfoil(Vox3DVoxelColliderData.AIRFOIL_SYMMETRIC, cx, cy, cz, nx, ny, nz, 1.0, 6.0);
        }

        return entry;
    }

    private static boolean isSail(final BlockState state) {
        if (state.getBlock() instanceof BlockSubLevelLiftProvider) {
            return true;
        }
        final String name = state.getBlock().getDescriptionId().toLowerCase();
        return name.contains("sail");
    }

    private static Direction getSailNormal(final BlockState state) {
        if (state.getBlock() instanceof final BlockSubLevelLiftProvider liftProvider) {
            return liftProvider.sable$getNormal(state);
        }
        if (state.hasProperty(BlockStateProperties.FACING)) {
            return state.getValue(BlockStateProperties.FACING);
        }
        if (state.hasProperty(BlockStateProperties.HORIZONTAL_FACING)) {
            return state.getValue(BlockStateProperties.HORIZONTAL_FACING);
        }
        return Direction.UP;
    }

    public @Nullable Vox3DVoxelColliderData getPhysicsDataForBlock(final BlockState state) {
        final Vox3DVoxelColliderData data = this.blockPhysicsDataBuilder.apply(Objects.requireNonNull(state, "state"));
        return data == Vox3DVoxelColliderData.EMPTY ? null : data;
    }
}
