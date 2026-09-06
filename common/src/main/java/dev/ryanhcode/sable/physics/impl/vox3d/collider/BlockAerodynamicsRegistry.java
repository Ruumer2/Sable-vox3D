package dev.ryanhcode.sable.physics.impl.vox3d.collider;

import dev.ryanhcode.sable.api.block.BlockSubLevelLiftProvider;
import net.minecraft.core.Direction;
import net.minecraft.core.registries.BuiltInRegistries;
import net.minecraft.resources.ResourceLocation;
import net.minecraft.world.level.block.state.BlockState;
import net.minecraft.world.level.block.state.properties.BlockStateProperties;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

/**
 * Hardcoded registry of blocks and their aerodynamic shape definitions.
 * Provides pre-configured profiles for major mods (Create, Create Aeronautics,
 * Clockwork, Eureka, Vanilla) with fallback support for custom lift providers.
 */
public final class BlockAerodynamicsRegistry {

    public enum NormalMode {
        FACING_OPPOSITE,
        FACING,
        HORIZONTAL_FACING,
        AXIS,
        UP,
        AUTO
    }

    public record AerodynamicDefinition(
            int airfoilType,
            double area,
            double aspectRatio,
            @NotNull NormalMode normalMode
    ) {
        public static final AerodynamicDefinition DEFAULT_SAIL =
                new AerodynamicDefinition(Vox3DVoxelColliderData.AIRFOIL_SYMMETRIC, 1.0, 6.0, NormalMode.FACING_OPPOSITE);

        public static final AerodynamicDefinition DEFAULT_WING =
                new AerodynamicDefinition(Vox3DVoxelColliderData.AIRFOIL_SYMMETRIC, 1.0, 6.0, NormalMode.AUTO);

        public static final AerodynamicDefinition DEFAULT_FLAT_PLATE =
                new AerodynamicDefinition(Vox3DVoxelColliderData.AIRFOIL_FLAT_PLATE, 1.0, 5.0, NormalMode.AUTO);

        public static final AerodynamicDefinition DEFAULT_CAMBERED_WING =
                new AerodynamicDefinition(Vox3DVoxelColliderData.AIRFOIL_CAMBERED, 1.0, 6.0, NormalMode.AUTO);

        public static final AerodynamicDefinition DEFAULT_BALLOON =
                new AerodynamicDefinition(Vox3DVoxelColliderData.AIRFOIL_FLAT_PLATE, 1.0, 4.0, NormalMode.UP);
    }

    private static final Map<String, AerodynamicDefinition> HARDCODED_BLOCKS;

    static {
        final Map<String, AerodynamicDefinition> map = new HashMap<>();

        // ==========================================
        // 1. Create Mod: Sails & Frames (All 16 Colors)
        // ==========================================
        map.put("create:white_sail", AerodynamicDefinition.DEFAULT_SAIL);
        map.put("create:sail_frame", AerodynamicDefinition.DEFAULT_SAIL);
        map.put("create:black_sail", AerodynamicDefinition.DEFAULT_SAIL);
        map.put("create:blue_sail", AerodynamicDefinition.DEFAULT_SAIL);
        map.put("create:brown_sail", AerodynamicDefinition.DEFAULT_SAIL);
        map.put("create:cyan_sail", AerodynamicDefinition.DEFAULT_SAIL);
        map.put("create:gray_sail", AerodynamicDefinition.DEFAULT_SAIL);
        map.put("create:green_sail", AerodynamicDefinition.DEFAULT_SAIL);
        map.put("create:light_blue_sail", AerodynamicDefinition.DEFAULT_SAIL);
        map.put("create:light_gray_sail", AerodynamicDefinition.DEFAULT_SAIL);
        map.put("create:lime_sail", AerodynamicDefinition.DEFAULT_SAIL);
        map.put("create:magenta_sail", AerodynamicDefinition.DEFAULT_SAIL);
        map.put("create:orange_sail", AerodynamicDefinition.DEFAULT_SAIL);
        map.put("create:pink_sail", AerodynamicDefinition.DEFAULT_SAIL);
        map.put("create:purple_sail", AerodynamicDefinition.DEFAULT_SAIL);
        map.put("create:red_sail", AerodynamicDefinition.DEFAULT_SAIL);
        map.put("create:yellow_sail", AerodynamicDefinition.DEFAULT_SAIL);

        // ==========================================
        // 2. Create: Aeronautics
        // ==========================================
        map.put("aeronautics:symmetric_sail", AerodynamicDefinition.DEFAULT_SAIL);
        map.put("aeronautics:windmill_sail", new AerodynamicDefinition(Vox3DVoxelColliderData.AIRFOIL_CAMBERED, 1.0, 6.0, NormalMode.FACING_OPPOSITE));
        map.put("aeronautics:wing", AerodynamicDefinition.DEFAULT_WING);
        map.put("aeronautics:wing_flap", AerodynamicDefinition.DEFAULT_WING);
        map.put("aeronautics:rudder", AerodynamicDefinition.DEFAULT_WING);
        map.put("aeronautics:aileron", AerodynamicDefinition.DEFAULT_WING);
        map.put("aeronautics:elevator", AerodynamicDefinition.DEFAULT_WING);
        map.put("aeronautics:stabilizer", AerodynamicDefinition.DEFAULT_WING);
        map.put("aeronautics:vertical_stabilizer", AerodynamicDefinition.DEFAULT_WING);
        map.put("aeronautics:horizontal_stabilizer", AerodynamicDefinition.DEFAULT_WING);
        map.put("aeronautics:propeller", AerodynamicDefinition.DEFAULT_CAMBERED_WING);
        map.put("aeronautics:propeller_blade", AerodynamicDefinition.DEFAULT_CAMBERED_WING);

        // ==========================================
        // 3. Create: Air Warfare & Addons
        // ==========================================
        map.put("create_air_warfare:wing", AerodynamicDefinition.DEFAULT_WING);
        map.put("create_air_warfare:flap", AerodynamicDefinition.DEFAULT_WING);
        map.put("create_air_warfare:aileron", AerodynamicDefinition.DEFAULT_WING);
        map.put("create_air_warfare:rudder", AerodynamicDefinition.DEFAULT_WING);
        map.put("create_air_warfare:elevator", AerodynamicDefinition.DEFAULT_WING);
        map.put("create_air_warfare:stabilizer", AerodynamicDefinition.DEFAULT_WING);
        map.put("create_air_warfare:sail", AerodynamicDefinition.DEFAULT_SAIL);

        // ==========================================
        // 4. Valkyrien Skies / Clockwork / Eureka
        // ==========================================
        map.put("vs_clockwork:wing", AerodynamicDefinition.DEFAULT_WING);
        map.put("vs_clockwork:flap", AerodynamicDefinition.DEFAULT_WING);
        map.put("vs_clockwork:rudder", AerodynamicDefinition.DEFAULT_WING);
        map.put("vs_clockwork:aileron", AerodynamicDefinition.DEFAULT_WING);
        map.put("vs_clockwork:elevator", AerodynamicDefinition.DEFAULT_WING);
        map.put("vs_clockwork:balloon", AerodynamicDefinition.DEFAULT_BALLOON);
        map.put("vs_clockwork:propellor", AerodynamicDefinition.DEFAULT_CAMBERED_WING);
        map.put("vs_clockwork:propeller", AerodynamicDefinition.DEFAULT_CAMBERED_WING);
        map.put("vs_eureka:sail", AerodynamicDefinition.DEFAULT_SAIL);
        map.put("vs_eureka:balloon", AerodynamicDefinition.DEFAULT_BALLOON);
        map.put("valkyrienskies:wing", AerodynamicDefinition.DEFAULT_WING);
        map.put("valkyrienskies:flap", AerodynamicDefinition.DEFAULT_WING);

        // ==========================================
        // 5. Vanilla Minecraft Banners & Gliders
        // ==========================================
        final String[] colors = {
                "white", "orange", "magenta", "light_blue", "yellow", "lime", "pink", "gray",
                "light_gray", "cyan", "purple", "blue", "brown", "green", "red", "black"
        };
        for (final String color : colors) {
            map.put("minecraft:" + color + "_banner", AerodynamicDefinition.DEFAULT_FLAT_PLATE);
            map.put("minecraft:" + color + "_wall_banner", AerodynamicDefinition.DEFAULT_FLAT_PLATE);
        }
        map.put("minecraft:elytra", AerodynamicDefinition.DEFAULT_WING);

        HARDCODED_BLOCKS = Collections.unmodifiableMap(map);
    }

    private BlockAerodynamicsRegistry() {}

    /**
     * Looks up the aerodynamic profile for a given block state.
     * Checks the hardcoded block list first, then falls back to Sable lift provider
     * interfaces and keyword matching.
     */
    public static @Nullable AerodynamicDefinition getAerodynamics(@NotNull final BlockState state) {
        final ResourceLocation key = BuiltInRegistries.BLOCK.getKey(state.getBlock());
        final String blockId = key.toString();

        final AerodynamicDefinition exact = HARDCODED_BLOCKS.get(blockId);
        if (exact != null) {
            return exact;
        }

        // Check if block implements Sable's lift provider interface
        if (state.getBlock() instanceof BlockSubLevelLiftProvider) {
            return AerodynamicDefinition.DEFAULT_SAIL;
        }

        // Keyword fallback matching on block path
        final String path = key.getPath().toLowerCase();
        if (path.contains("sail")) {
            return AerodynamicDefinition.DEFAULT_SAIL;
        }
        if (path.contains("wing") || path.contains("flap") || path.contains("rudder") ||
                path.contains("aileron") || path.contains("elevator") || path.contains("stabilizer") ||
                path.contains("airfoil")) {
            return AerodynamicDefinition.DEFAULT_WING;
        }
        if (path.contains("propeller") || path.contains("propellor")) {
            return AerodynamicDefinition.DEFAULT_CAMBERED_WING;
        }
        if (path.contains("balloon")) {
            return AerodynamicDefinition.DEFAULT_BALLOON;
        }
        if (path.contains("banner")) {
            return AerodynamicDefinition.DEFAULT_FLAT_PLATE;
        }

        return null;
    }

    /**
     * Extracts the surface normal direction for an aerodynamic block.
     */
    public static @NotNull Direction getNormal(@NotNull final BlockState state, @NotNull final AerodynamicDefinition def) {
        if (state.getBlock() instanceof final BlockSubLevelLiftProvider liftProvider) {
            return liftProvider.sable$getNormal(state);
        }

        return switch (def.normalMode()) {
            case FACING_OPPOSITE -> {
                if (state.hasProperty(BlockStateProperties.FACING)) {
                    yield state.getValue(BlockStateProperties.FACING).getOpposite();
                }
                if (state.hasProperty(BlockStateProperties.HORIZONTAL_FACING)) {
                    yield state.getValue(BlockStateProperties.HORIZONTAL_FACING).getOpposite();
                }
                yield Direction.UP;
            }
            case FACING -> {
                if (state.hasProperty(BlockStateProperties.FACING)) {
                    yield state.getValue(BlockStateProperties.FACING);
                }
                if (state.hasProperty(BlockStateProperties.HORIZONTAL_FACING)) {
                    yield state.getValue(BlockStateProperties.HORIZONTAL_FACING);
                }
                yield Direction.UP;
            }
            case HORIZONTAL_FACING -> {
                if (state.hasProperty(BlockStateProperties.HORIZONTAL_FACING)) {
                    yield state.getValue(BlockStateProperties.HORIZONTAL_FACING);
                }
                yield Direction.NORTH;
            }
            case AXIS -> {
                if (state.hasProperty(BlockStateProperties.AXIS)) {
                    yield Direction.fromAxisAndDirection(state.getValue(BlockStateProperties.AXIS), Direction.AxisDirection.POSITIVE);
                }
                yield Direction.UP;
            }
            case UP -> Direction.UP;
            case AUTO -> {
                final ResourceLocation key = BuiltInRegistries.BLOCK.getKey(state.getBlock());
                final String path = key.getPath().toLowerCase();
                if (path.contains("sail")) {
                    if (state.hasProperty(BlockStateProperties.FACING)) {
                        yield state.getValue(BlockStateProperties.FACING).getOpposite();
                    }
                    if (state.hasProperty(BlockStateProperties.HORIZONTAL_FACING)) {
                        yield state.getValue(BlockStateProperties.HORIZONTAL_FACING).getOpposite();
                    }
                } else {
                    if (state.hasProperty(BlockStateProperties.FACING)) {
                        yield state.getValue(BlockStateProperties.FACING);
                    }
                    if (state.hasProperty(BlockStateProperties.HORIZONTAL_FACING)) {
                        yield state.getValue(BlockStateProperties.HORIZONTAL_FACING);
                    }
                }
                if (state.hasProperty(BlockStateProperties.AXIS)) {
                    yield Direction.fromAxisAndDirection(state.getValue(BlockStateProperties.AXIS), Direction.AxisDirection.POSITIVE);
                }
                yield Direction.UP;
            }
        };
    }
}
