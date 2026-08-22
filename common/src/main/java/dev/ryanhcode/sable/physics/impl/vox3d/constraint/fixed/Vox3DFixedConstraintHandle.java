package dev.ryanhcode.sable.physics.impl.vox3d.constraint.fixed;

import dev.ryanhcode.sable.api.physics.PhysicsPipelineBody;
import dev.ryanhcode.sable.api.physics.constraint.FixedConstraintConfiguration;
import dev.ryanhcode.sable.api.physics.constraint.FixedConstraintHandle;
import dev.ryanhcode.sable.physics.impl.vox3d.Vox3D;
import dev.ryanhcode.sable.physics.impl.vox3d.constraint.Vox3DConstraintHandle;
import org.jetbrains.annotations.Nullable;

public class Vox3DFixedConstraintHandle extends Vox3DConstraintHandle implements FixedConstraintHandle {

    public static Vox3DFixedConstraintHandle create(final long sceneHandle, @Nullable final PhysicsPipelineBody bodyA, @Nullable final PhysicsPipelineBody bodyB, final FixedConstraintConfiguration config) {
        final long handle = Vox3D.addFixedConstraint(
                sceneHandle,
                Vox3D.getID(bodyA),
                Vox3D.getID(bodyB),
                config.pos1().x(),
                config.pos1().y(),
                config.pos1().z(),
                config.pos2().x(),
                config.pos2().y(),
                config.pos2().z(),
                config.orientation().x(),
                config.orientation().y(),
                config.orientation().z(),
                config.orientation().w()
        );

        return new Vox3DFixedConstraintHandle(sceneHandle, handle);
    }

    public Vox3DFixedConstraintHandle(final long sceneHandle, final long handle) {
        super(sceneHandle, handle);
    }
}
