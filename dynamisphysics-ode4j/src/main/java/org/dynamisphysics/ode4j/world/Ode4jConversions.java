package org.dynamisphysics.ode4j.world;

import org.ode4j.math.DQuaternion;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.vectrix.core.Quaternionf;
import org.vectrix.core.Vector3f;

public final class Ode4jConversions {
    private Ode4jConversions() {}

    public static DVector3 toOde(Vector3f v) {
        return new DVector3(v.x(), v.y(), v.z());
    }

    public static DQuaternion toOde(Quaternionf q) {
        return new DQuaternion(q.w(), q.x(), q.y(), q.z());
    }

    public static Vector3f toVec3f(DVector3C v) {
        return new Vector3f((float) v.get0(), (float) v.get1(), (float) v.get2());
    }
}
