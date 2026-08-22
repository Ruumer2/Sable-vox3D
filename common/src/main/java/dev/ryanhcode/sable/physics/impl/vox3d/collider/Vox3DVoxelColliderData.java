package dev.ryanhcode.sable.physics.impl.vox3d.collider;

import dev.ryanhcode.sable.api.physics.callback.BlockSubLevelCollisionCallback;
import dev.ryanhcode.sable.api.physics.collider.VoxelColliderData;
import dev.ryanhcode.sable.physics.impl.vox3d.Vox3D;
import org.joml.Vector3dc;

/**
 * Represents a block physics data entry in the Vox3D physics world.
 */
public record Vox3DVoxelColliderData(int handle, boolean isFluid, BlockSubLevelCollisionCallback callback) implements VoxelColliderData {
    public static final Vox3DVoxelColliderData EMPTY = new Vox3DVoxelColliderData(-1, false, null);

    public Vox3DVoxelColliderData(final int handle) {
        this(handle, false, null);
    }

    @Override
    public void addBox(final Vector3dc min, final Vector3dc max) {
        if (this.handle < 0) return;
        Vox3D.addVoxelColliderBox(this.handle, new double[]{min.x(), min.y(), min.z(), max.x(), max.y(), max.z()});
    }

    @Override
    public void clearBoxes() {
        if (this.handle < 0) return;
        Vox3D.clearVoxelColliderBoxes(this.handle);
    }
}
