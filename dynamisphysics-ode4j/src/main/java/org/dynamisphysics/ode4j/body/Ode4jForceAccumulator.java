package org.dynamisphysics.ode4j.body;

import org.ode4j.math.DVector3;
import org.ode4j.ode.DBody;
import org.vectrix.core.Vector3f;

import java.util.ArrayList;
import java.util.List;

import static org.dynamisphysics.ode4j.world.Ode4jConversions.toOde;

public final class Ode4jForceAccumulator {
    private record ImpulseEntry(DBody body, DVector3 impulse, DVector3 point) {}
    private record ForceEntry(DBody body, DVector3 force, DVector3 point) {}
    private record TorqueEntry(DBody body, DVector3 torque) {}
    private record VelEntry(DBody body, DVector3 linear, DVector3 angular) {}

    private final List<ImpulseEntry> impulses = new ArrayList<>();
    private final List<ForceEntry> forces = new ArrayList<>();
    private final List<TorqueEntry> torques = new ArrayList<>();
    private final List<VelEntry> vels = new ArrayList<>();

    public void addImpulse(DBody body, Vector3f impulse, Vector3f point) {
        impulses.add(new ImpulseEntry(body, toOde(impulse), toOde(point)));
    }

    public void addForce(DBody body, Vector3f force, Vector3f point) {
        forces.add(new ForceEntry(body, toOde(force), toOde(point)));
    }

    public void addTorque(DBody body, Vector3f torque) {
        torques.add(new TorqueEntry(body, toOde(torque)));
    }

    public void setVelocity(DBody body, Vector3f linear, Vector3f angular) {
        vels.add(new VelEntry(body, toOde(linear), toOde(angular)));
    }

    public void flush() {
        impulses.forEach(e -> e.body().addForceAtPos(e.impulse(), e.point()));
        forces.forEach(e -> e.body().addForceAtPos(e.force(), e.point()));
        torques.forEach(e -> e.body().addTorque(e.torque()));
        vels.forEach(e -> {
            e.body().setLinearVel(e.linear());
            e.body().setAngularVel(e.angular());
        });
        impulses.clear();
        forces.clear();
        torques.clear();
        vels.clear();
    }
}
