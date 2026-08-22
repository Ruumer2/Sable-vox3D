package dev.ryanhcode.sable.physics.impl.vox3d;

import dev.ryanhcode.sable.api.physics.PhysicsPipeline;
import dev.ryanhcode.sable.api.physics.PhysicsPipelineProvider;
import net.minecraft.server.level.ServerLevel;
import org.jetbrains.annotations.NotNull;

// Prefer Vox3D over Sable's built-in provider while still allowing a deliberately
// higher-priority addon to select its own backend through Sable's normal mechanism.
@PhysicsPipelineProvider.LoadPriority(1100)
public final class Vox3DPhysicsPipelineProvider implements PhysicsPipelineProvider {

    @Override
    public @NotNull PhysicsPipeline createPipeline(@NotNull final ServerLevel level) {
        return new Vox3DPhysicsPipeline(level);
    }
}
