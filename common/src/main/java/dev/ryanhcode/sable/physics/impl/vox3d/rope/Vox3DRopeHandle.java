package dev.ryanhcode.sable.physics.impl.vox3d.rope;

import dev.ryanhcode.sable.api.physics.object.rope.RopeHandle;
import dev.ryanhcode.sable.physics.impl.vox3d.Vox3D;
import dev.ryanhcode.sable.sublevel.ServerSubLevel;
import org.jetbrains.annotations.ApiStatus;
import org.joml.Vector3d;
import org.joml.Vector3dc;

import java.util.List;

@ApiStatus.Internal
public record Vox3DRopeHandle(long sceneHandle, long handle) implements RopeHandle {

    public static Vox3DRopeHandle create(final long sceneHandle, final double pointRadius, final List<Vector3d> points) {
        final double[] coordinates = new double[points.size() * 3];

        for (int i = 0; i < points.size(); i++) {
            final Vector3d point = points.get(i);
            coordinates[i * 3] = point.x;
            coordinates[i * 3 + 1] = point.y;
            coordinates[i * 3 + 2] = point.z;
        }

        final double firstDist = points.size() > 1 ? points.get(0).distance(points.get(1)) : 0.1;
        final long handle = Vox3D.createRope(sceneHandle, pointRadius, firstDist, coordinates, points.size());
        return new Vox3DRopeHandle(sceneHandle, handle);
    }

    @Override
    public void readPose(final List<Vector3d> dest) {
        final double[] coordinates = Vox3D.queryRope(this.sceneHandle, this.handle);
        for (int i = 0; i < coordinates.length && (i / 3) < dest.size(); i += 3) {
            dest.get(i / 3).set(coordinates[i], coordinates[i + 1], coordinates[i + 2]);
        }
    }

    @Override
    public void remove() {
        Vox3D.removeRope(this.sceneHandle, this.handle);
    }

    @Override
    public void setFirstSegmentLength(final double length) {
        Vox3D.setRopeFirstSegmentLength(this.sceneHandle, this.handle, length);
    }

    @Override
    public void removeFirstPoint() {
        Vox3D.removeRopePointAtStart(this.sceneHandle, this.handle);
    }

    @Override
    public void addPoint(final Vector3dc position) {
        Vox3D.addRopePointAtStart(this.sceneHandle, this.handle, position.x(), position.y(), position.z());
    }

    @Override
    public void setAttachment(final AttachmentPoint attachmentPoint, final Vector3dc location, final ServerSubLevel subLevel) {
        Vox3D.setRopeAttachment(this.sceneHandle, this.handle, Vox3D.getID(subLevel), location.x(), location.y(), location.z(), attachmentPoint == AttachmentPoint.END);
    }

    @Override
    public void wakeUp() {
        Vox3D.wakeUpRope(this.sceneHandle, this.handle);
    }
}
