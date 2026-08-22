package dev.ryanhcode.sable.physics.impl.vox3d;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class SableVox3DMod {
    public static final String MOD_ID = "sable_vox3d";
    public static final String MOD_NAME = "Sable Vox3D";
    public static final Logger LOGGER = LoggerFactory.getLogger(MOD_NAME);

    public static void init() {
        LOGGER.info("Initializing Sable Vox3D (Box3D physics engine addon)...");
    }
}
