package dev.ryanhcode.sable.physics.impl.vox3d.constraint.generic;

import dev.ryanhcode.sable.api.physics.PhysicsPipelineBody;
import dev.ryanhcode.sable.api.physics.constraint.ConstraintJointAxis;
import dev.ryanhcode.sable.api.physics.constraint.GenericConstraintConfiguration;
import dev.ryanhcode.sable.api.physics.constraint.GenericConstraintHandle;
import dev.ryanhcode.sable.physics.impl.vox3d.Vox3D;
import dev.ryanhcode.sable.physics.impl.vox3d.constraint.Vox3DConstraintHandle;
import org.jetbrains.annotations.ApiStatus;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import org.joml.Quaterniondc;
import org.joml.Vector3dc;

@ApiStatus.Internal
public class Vox3DGenericConstraintHandle extends Vox3DConstraintHandle implements GenericConstraintHandle {

    private static final int FRAME_SIDE_FIRST = 0;
    private static final int FRAME_SIDE_SECOND = 1;
    private int lockedAxesMask;

    private static boolean isSupportedMask(final int mask) {
        final int normalized = mask & 0x3F;
        final int linearMask = normalized & 0x7;
        final int angularMask = (normalized >>> 3) & 0x7;
        final int freeLinearMask = (~linearMask) & 0x7;
        final boolean exactlyOneFreeLinearAxis = freeLinearMask == 1 || freeLinearMask == 2 || freeLinearMask == 4;
        return normalized == 0 || linearMask == 0x7 ||
                (angularMask == 0x7 && exactlyOneFreeLinearAxis);
    }

    @Contract("_, _, _, _ -> new")
    public static @NotNull Vox3DGenericConstraintHandle create(final long sceneHandle, @Nullable final PhysicsPipelineBody bodyA, @Nullable final PhysicsPipelineBody bodyB, final GenericConstraintConfiguration config) {
        int lockedAxesMask = 0;
        for (final ConstraintJointAxis axis : config.lockedAxes()) {
            lockedAxesMask |= 1 << axis.ordinal();
        }
        if (!isSupportedMask(lockedAxesMask)) {
            throw new UnsupportedOperationException(
                    "Vox3D cannot exactly represent GenericConstraint locked-axis mask 0x" +
                            Integer.toHexString(lockedAxesMask) +
                            "; use a spherical, revolute, prismatic, weld, or free mask");
        }

        final long handle = Vox3D.addGenericConstraint(
                sceneHandle,
                Vox3D.getID(bodyA),
                Vox3D.getID(bodyB),
                config.pos1().x(),
                config.pos1().y(),
                config.pos1().z(),
                config.orientation1().x(),
                config.orientation1().y(),
                config.orientation1().z(),
                config.orientation1().w(),
                config.pos2().x(),
                config.pos2().y(),
                config.pos2().z(),
                config.orientation2().x(),
                config.orientation2().y(),
                config.orientation2().z(),
                config.orientation2().w(),
                lockedAxesMask
        );

        return new Vox3DGenericConstraintHandle(sceneHandle, handle, lockedAxesMask);
    }

    public Vox3DGenericConstraintHandle(final long sceneHandle, final long handle) {
        this(sceneHandle, handle, 0);
    }

    private Vox3DGenericConstraintHandle(final long sceneHandle, final long handle, final int lockedAxesMask) {
        super(sceneHandle, handle);
        this.lockedAxesMask = lockedAxesMask;
    }

    @Override
    public void setFrame1(final Vector3dc localPosition, final Quaterniondc localOrientation) {
        this.assertValid();
        Vox3D.setConstraintFrame(
                this.sceneHandle, this.handle, FRAME_SIDE_FIRST,
                localPosition.x(), localPosition.y(), localPosition.z(),
                localOrientation.x(), localOrientation.y(), localOrientation.z(), localOrientation.w()
        );
    }

    @Override
    public void setFrame2(final Vector3dc localPosition, final Quaterniondc localOrientation) {
        this.assertValid();
        Vox3D.setConstraintFrame(
                this.sceneHandle, this.handle, FRAME_SIDE_SECOND,
                localPosition.x(), localPosition.y(), localPosition.z(),
                localOrientation.x(), localOrientation.y(), localOrientation.z(), localOrientation.w()
        );
    }

    @Override
    public void setLimit(final ConstraintJointAxis axis, final double min, final double max) {
        final int linearMask = this.lockedAxesMask & 0x7;
        final int angularMask = (this.lockedAxesMask >>> 3) & 0x7;
        final int requestedBit = 1 << axis.ordinal();
        final boolean revoluteLimit = linearMask == 0x7 && Integer.bitCount(angularMask) == 2 &&
                axis.ordinal() >= 3 && (((~this.lockedAxesMask) & 0x3F) & requestedBit) != 0;
        final boolean prismaticLimit = angularMask == 0x7 && Integer.bitCount(linearMask) == 2 &&
                axis.ordinal() < 3 && (((~this.lockedAxesMask) & 0x3F) & requestedBit) != 0;
        if (!revoluteLimit && !prismaticLimit) {
            throw new UnsupportedOperationException(
                    "Vox3D can apply GenericConstraint limits only to the single free axis of an exact " +
                            "revolute or prismatic mapping");
        }
        this.assertValid();
        Vox3D.setConstraintLimit(this.sceneHandle, this.handle, axis.ordinal(), min, max);
    }

    @Override
    public void lockAxes(final ConstraintJointAxis @NotNull... axes) {
        byte mask = 0;
        for (final ConstraintJointAxis axis : axes) {
            final byte bit = (byte) (1 << axis.ordinal());
            if ((mask & bit) != 0) {
                throw new RuntimeException("Duplicate axis: " + axis);
            }
            mask |= bit;
        }

        final int newMask = this.lockedAxesMask | (mask & 0x3F);
        if (!isSupportedMask(newMask)) {
            throw new UnsupportedOperationException(
                    "Vox3D cannot exactly represent GenericConstraint locked-axis mask 0x" +
                            Integer.toHexString(newMask) +
                            "; pass all axes for a supported joint shape in one call");
        }

        this.assertValid();
        if (newMask == this.lockedAxesMask) {
            return;
        }
        if (!Vox3D.lockConstraintAxes(this.sceneHandle, this.handle, mask)) {
            throw new IllegalStateException("Vox3D rejected GenericConstraint axis mask update");
        }
        this.lockedAxesMask = newMask;
    }
}
