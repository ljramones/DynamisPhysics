package org.dynamisphysics.jolt.world;

import com.github.stephengold.joltjni.Quat;
import com.github.stephengold.joltjni.RVec3;
import com.github.stephengold.joltjni.Vec3;
import org.vectrix.core.Quaternionf;
import org.vectrix.core.Vector3f;

public final class JoltConversions {
    private JoltConversions() {
    }

    public static Vec3 toVec3(Vector3f v) {
        return new Vec3(v.x(), v.y(), v.z());
    }

    public static RVec3 toRVec3(Vector3f v) {
        return new RVec3(v.x(), v.y(), v.z());
    }

    public static Quat toQuat(Quaternionf q) {
        return new Quat(q.x(), q.y(), q.z(), q.w());
    }

    public static Vector3f toVector3f(Vec3 v) {
        return new Vector3f(v.getX(), v.getY(), v.getZ());
    }

    public static Vector3f toVector3f(RVec3 v) {
        return new Vector3f((float) v.xx(), (float) v.yy(), (float) v.zz());
    }

    public static Quaternionf toQuaternionf(Quat q) {
        return new Quaternionf(q.getX(), q.getY(), q.getZ(), q.getW());
    }
}
