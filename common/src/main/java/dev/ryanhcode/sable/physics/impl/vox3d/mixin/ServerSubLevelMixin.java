package dev.ryanhcode.sable.physics.impl.vox3d.mixin;

import dev.ryanhcode.sable.api.block.BlockSubLevelLiftProvider;
import dev.ryanhcode.sable.sublevel.ServerSubLevel;
import dev.ryanhcode.sable.sublevel.plot.ServerLevelPlot;
import it.unimi.dsi.fastutil.objects.ObjectCollection;
import it.unimi.dsi.fastutil.objects.ObjectSets;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Redirect;

@Mixin(value = ServerSubLevel.class, remap = false)
public class ServerSubLevelMixin {

    /**
     * Redirect getLiftProviders in ServerSubLevel.prePhysicsTick to return an empty set,
     * allowing Vox3D's shape-based aerodynamics to fully govern flight dynamics.
     */
    @Redirect(
            method = "prePhysicsTick",
            at = @At(
                    value = "INVOKE",
                    target = "Ldev/ryanhcode/sable/sublevel/plot/ServerLevelPlot;getLiftProviders()Lit/unimi/dsi/fastutil/objects/ObjectCollection;"
            )
    )
    private ObjectCollection<BlockSubLevelLiftProvider.LiftProviderContext> sable_vox3d$emptyLiftProviders(final ServerLevelPlot instance) {
        return ObjectSets.emptySet();
    }
}
