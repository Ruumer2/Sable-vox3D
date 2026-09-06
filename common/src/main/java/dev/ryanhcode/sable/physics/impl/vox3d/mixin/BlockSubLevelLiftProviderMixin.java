package dev.ryanhcode.sable.physics.impl.vox3d.mixin;

import dev.ryanhcode.sable.api.block.BlockSubLevelLiftProvider;
import dev.ryanhcode.sable.companion.math.Pose3d;
import dev.ryanhcode.sable.sublevel.ServerSubLevel;
import org.joml.Vector3d;
import org.joml.Vector3dc;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Inject;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfo;

@Mixin(value = BlockSubLevelLiftProvider.class, remap = false)
public interface BlockSubLevelLiftProviderMixin {

    /**
     * Completely overwrite and neutralize the legacy Java-based lift and drag calculation
     * from Sable/Aeronautics. Vox3D calculates geometric shape-based aerodynamics natively in C.
     */
    @Inject(method = "sable$contributeLiftAndDrag", at = @At("HEAD"), cancellable = true)
    default void sable_vox3d$cancelLegacyLiftAndDrag(
            final BlockSubLevelLiftProvider.LiftProviderContext ctx,
            final ServerSubLevel subLevel,
            final Pose3d localPose,
            final double timeStep,
            final Vector3dc linearVelocity,
            final Vector3dc angularVelocity,
            final Vector3d linearImpulse,
            final Vector3d angularImpulse,
            final BlockSubLevelLiftProvider.LiftProviderGroup group,
            final CallbackInfo ci
    ) {
        ci.cancel();
    }
}
