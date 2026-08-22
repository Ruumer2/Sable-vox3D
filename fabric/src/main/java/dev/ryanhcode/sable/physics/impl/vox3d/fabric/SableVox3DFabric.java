package dev.ryanhcode.sable.physics.impl.vox3d.fabric;

import dev.ryanhcode.sable.physics.impl.vox3d.SableVox3DMod;
import net.fabricmc.api.ModInitializer;

public class SableVox3DFabric implements ModInitializer {
    @Override
    public void onInitialize() {
        SableVox3DMod.init();
    }
}
