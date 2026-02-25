package org.dynamisphysics.api.world;

import org.dynamiscollision.shapes.CollisionShape;
import org.dynamisphysics.api.AnimisPose;
import org.dynamisphysics.api.CharacterDescriptor;
import org.dynamisphysics.api.CharacterHandle;
import org.dynamisphysics.api.RagdollDescriptor;
import org.dynamisphysics.api.RagdollHandle;
import org.dynamisphysics.api.VehicleDescriptor;
import org.dynamisphysics.api.VehicleHandle;
import org.dynamisphysics.api.body.BodyState;
import org.dynamisphysics.api.body.RigidBodyConfig;
import org.dynamisphysics.api.body.RigidBodyHandle;
import org.dynamisphysics.api.constraint.ConstraintDesc;
import org.dynamisphysics.api.constraint.ConstraintHandle;
import org.dynamisphysics.api.event.ContactListener;
import org.dynamisphysics.api.event.PhysicsEvent;
import org.dynamisphysics.api.query.RaycastResult;
import org.dynamisphysics.api.query.ShapecastResult;
import org.vectrix.core.Quaternionf;
import org.vectrix.core.Vector3f;

import java.util.List;
import java.util.Optional;

public interface PhysicsWorld {
    void step(float deltaSeconds);
    void step(float deltaSeconds, int subSteps);
    void pause();
    void resume();
    void destroy();

    RigidBodyHandle spawnRigidBody(RigidBodyConfig config);
    void destroyRigidBody(RigidBodyHandle handle);
    BodyState getBodyState(RigidBodyHandle handle);
    void setBodyState(RigidBodyHandle handle, BodyState state);

    void applyImpulse(RigidBodyHandle h, Vector3f impulse, Vector3f worldPoint);
    void applyForce(RigidBodyHandle h, Vector3f force, Vector3f worldPoint);
    void applyTorque(RigidBodyHandle h, Vector3f torque);
    void setVelocity(RigidBodyHandle h, Vector3f linear, Vector3f angular);
    void teleport(RigidBodyHandle h, Vector3f position, Quaternionf orientation);

    ConstraintHandle addConstraint(ConstraintDesc desc);
    void removeConstraint(ConstraintHandle handle);
    void setConstraintEnabled(ConstraintHandle h, boolean enabled);
    void setMotorTarget(ConstraintHandle h, float targetVelocityOrPosition);

    VehicleHandle spawnVehicle(VehicleDescriptor desc);
    void destroyVehicle(VehicleHandle handle);
    void applyThrottle(VehicleHandle h, float throttle);
    void applyBrake(VehicleHandle h, float brake);
    void applySteering(VehicleHandle h, float steeringAngle);
    void applyHandbrake(VehicleHandle h, boolean engaged);
    VehicleState getVehicleState(VehicleHandle h);

    CharacterHandle spawnCharacter(CharacterDescriptor desc);
    void destroyCharacter(CharacterHandle handle);
    void moveCharacter(CharacterHandle h, Vector3f velocity);
    void jumpCharacter(CharacterHandle h, float impulse);
    CharacterState getCharacterState(CharacterHandle h);

    RagdollHandle spawnRagdoll(RagdollDescriptor desc, AnimisPose initialPose);
    void destroyRagdoll(RagdollHandle handle);
    void activateRagdoll(RagdollHandle h, float blendInSeconds);
    void deactivateRagdoll(RagdollHandle h);
    void setRagdollBlendTarget(RagdollHandle h, AnimisPose pose, float alpha);

    Optional<RaycastResult> raycastClosest(Vector3f origin, Vector3f dir, float maxDist, int layerMask);
    List<RaycastResult> raycastAll(Vector3f origin, Vector3f dir, float maxDist, int layerMask);
    List<RigidBodyHandle> overlapSphere(Vector3f centre, float radius, int layerMask);
    List<RigidBodyHandle> overlapAabb(Vector3f min, Vector3f max, int layerMask);
    Optional<ShapecastResult> shapecast(CollisionShape shape, Vector3f from, Vector3f to, int layerMask);

    void addContactListener(ContactListener listener);
    void removeContactListener(ContactListener listener);
    List<PhysicsEvent> drainEvents();

    byte[] snapshot();
    void restore(byte[] snapshot);

    void setGravity(Vector3f gravity);
    void setTimeScale(float scale);
    PhysicsStats getStats();
}
