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

    public static final int AIRFOIL_NONE = 0;
    public static final int AIRFOIL_FLAT_PLATE = 1;
    public static final int AIRFOIL_SYMMETRIC = 2;
    public static final int AIRFOIL_CAMBERED = 3;
    public static final int AIRFOIL_CUSTOM = 4;
    public static final int AIRFOIL_NACA_4412 = 5;

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

    public void setAirfoil(final int airfoilType, final Vector3dc chordAxis, final Vector3dc upAxis, final double area, final double aspectRatio) {
        if (this.handle < 0) return;
        Vox3D.setVoxelColliderAirfoil(this.handle, airfoilType, chordAxis.x(), chordAxis.y(), chordAxis.z(), upAxis.x(), upAxis.y(), upAxis.z(), area, aspectRatio);
    }

    public void setAirfoil(final int airfoilType, final double chordX, final double chordY, final double chordZ, final double upX, final double upY, final double upZ, final double area, final double aspectRatio) {
        if (this.handle < 0) return;
        Vox3D.setVoxelColliderAirfoil(this.handle, airfoilType, chordX, chordY, chordZ, upX, upY, upZ, area, aspectRatio);
    }
}
