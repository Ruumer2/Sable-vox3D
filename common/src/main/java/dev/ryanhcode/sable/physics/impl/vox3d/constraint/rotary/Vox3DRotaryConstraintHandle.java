package dev.ryanhcode.sable.physics.impl.vox3d.constraint.rotary;

import dev.ryanhcode.sable.api.physics.PhysicsPipelineBody;
import dev.ryanhcode.sable.api.physics.constraint.RotaryConstraintConfiguration;
import dev.ryanhcode.sable.api.physics.constraint.RotaryConstraintHandle;
import dev.ryanhcode.sable.physics.impl.vox3d.Vox3D;
import dev.ryanhcode.sable.physics.impl.vox3d.constraint.Vox3DConstraintHandle;
import org.jetbrains.annotations.Nullable;

public class Vox3DRotaryConstraintHandle extends Vox3DConstraintHandle implements RotaryConstraintHandle {

    public static Vox3DRotaryConstraintHandle create(final long sceneHandle, @Nullable final PhysicsPipelineBody bodyA, @Nullable final PhysicsPipelineBody bodyB, final RotaryConstraintConfiguration config) {
        final long handle = Vox3D.addRotaryConstraint(
                sceneHandle,
                Vox3D.getID(bodyA),
                Vox3D.getID(bodyB),
                config.pos1().x(),
                config.pos1().y(),
                config.pos1().z(),
                config.pos2().x(),
                config.pos2().y(),
                config.pos2().z(),
                config.normal1().x(),
                config.normal1().y(),
                config.normal1().z(),
                config.normal2().x(),
                config.normal2().y(),
                config.normal2().z()
        );

        return new Vox3DRotaryConstraintHandle(sceneHandle, handle);
    }

    public Vox3DRotaryConstraintHandle(final long sceneHandle, final long handle) {
        super(sceneHandle, handle);
    }
}
