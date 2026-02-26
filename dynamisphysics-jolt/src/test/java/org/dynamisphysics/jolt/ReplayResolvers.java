package org.dynamisphysics.jolt;

import org.dynamisphysics.api.CharacterHandle;
import org.dynamisphysics.api.RagdollHandle;
import org.dynamisphysics.api.VehicleHandle;
import org.dynamisphysics.api.body.RigidBodyHandle;
import org.dynamisphysics.api.config.PhysicsBackend;
import org.dynamisphysics.api.world.PhysicsWorld;
import org.dynamisphysics.ode4j.Ode4jPhysicsWorld;
import org.dynamisphysics.test.replay.PhysicsReplayRunner;

final class ReplayResolvers {
    private ReplayResolvers() {
    }

    static PhysicsReplayRunner.ReplayHandleResolver forWorld(PhysicsBackend backend, PhysicsWorld world) {
        return switch (backend) {
            case ODE4J -> fromOde((Ode4jPhysicsWorld) world);
            case JOLT -> fromJolt((JoltPhysicsWorld) world);
        };
    }

    private static PhysicsReplayRunner.ReplayHandleResolver fromOde(Ode4jPhysicsWorld world) {
        return new PhysicsReplayRunner.ReplayHandleResolver() {
            @Override
            public RigidBodyHandle rigidBody(int id) {
                RigidBodyHandle handle = world.resolveBodyById(id);
                if (handle == null) {
                    throw new IllegalArgumentException("ODE4J replay cannot resolve rigidBodyId=" + id);
                }
                return handle;
            }

            @Override
            public VehicleHandle vehicle(int id) {
                throw new UnsupportedOperationException("Vehicle replay resolution not implemented yet for ODE4J (id=" + id + ")");
            }

            @Override
            public CharacterHandle character(int id) {
                throw new UnsupportedOperationException("Character replay resolution not implemented yet for ODE4J (id=" + id + ")");
            }

            @Override
            public RagdollHandle ragdoll(int id) {
                throw new UnsupportedOperationException("Ragdoll replay resolution not implemented yet for ODE4J (id=" + id + ")");
            }
        };
    }

    private static PhysicsReplayRunner.ReplayHandleResolver fromJolt(JoltPhysicsWorld world) {
        return new PhysicsReplayRunner.ReplayHandleResolver() {
            @Override
            public RigidBodyHandle rigidBody(int id) {
                RigidBodyHandle handle = world.resolveBodyById(id);
                if (handle == null) {
                    throw new IllegalArgumentException("Jolt replay cannot resolve rigidBodyId=" + id);
                }
                return handle;
            }

            @Override
            public VehicleHandle vehicle(int id) {
                throw new UnsupportedOperationException("Vehicle replay resolution not implemented yet for Jolt (id=" + id + ")");
            }

            @Override
            public CharacterHandle character(int id) {
                throw new UnsupportedOperationException("Character replay resolution not implemented yet for Jolt (id=" + id + ")");
            }

            @Override
            public RagdollHandle ragdoll(int id) {
                throw new UnsupportedOperationException("Ragdoll replay resolution not implemented yet for Jolt (id=" + id + ")");
            }
        };
    }
}
