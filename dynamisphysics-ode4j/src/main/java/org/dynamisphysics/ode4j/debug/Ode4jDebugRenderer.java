package org.dynamisphysics.ode4j.debug;

import org.dynamisgpu.api.gpu.BoundsBuffer;
import org.dynamisgpu.api.gpu.StagingScheduler;
import org.dynamisgpu.api.gpu.VkCommandBuffer;
import org.dynamisphysics.api.body.BodyMode;
import org.dynamisphysics.api.body.BodyState;
import org.dynamisphysics.api.constraint.ConstraintDesc;
import org.dynamisphysics.api.world.PhysicsDebugConfig;
import org.dynamisphysics.ode4j.Ode4jPhysicsWorld;
import org.dynamisphysics.ode4j.body.Ode4jBodyHandle;
import org.dynamisphysics.ode4j.constraint.Ode4jConstraintHandle;
import org.dynamisphysics.ode4j.event.Ode4jContactDispatcher;
import org.vectrix.core.Vector3f;

import java.util.List;

public final class Ode4jDebugRenderer {
    private final WireframeBuilder wireframe;
    private final BoundsBuffer boundsBuffer;
    private final StagingScheduler staging;

    private int lastVertexCount;

    public Ode4jDebugRenderer(int maxLines) {
        this(maxLines, null, null);
    }

    public Ode4jDebugRenderer(int maxLines, BoundsBuffer boundsBuffer, StagingScheduler staging) {
        this.wireframe = new WireframeBuilder(maxLines);
        this.boundsBuffer = boundsBuffer;
        this.staging = staging;
    }

    public void update(Ode4jPhysicsWorld world, PhysicsDebugConfig cfg) {
        wireframe.reset();
        lastVertexCount = 0;

        if (!anyEnabled(cfg)) {
            uploadIfAvailable();
            return;
        }

        if (cfg.showShapes()) {
            for (Ode4jBodyHandle body : world.debugBodiesInIdOrder()) {
                BodyState state = world.getBodyState(body);
                float[] color = DebugColourPolicy.colourForBody(body, cfg.showSleepState());
                ShapeWireframes.addShape(wireframe, body.config().shape(), state.position(), state.orientation(), color, cfg.showAabbs());
            }
        }

        if (cfg.showConstraints()) {
            for (Ode4jConstraintHandle c : world.debugConstraintsInIdOrder()) {
                ConstraintDesc d = c.desc();
                float[] color = DebugColourPolicy.constraintColour();
                wireframe.addCross(d.pivotA(), 0.08f, color);
                wireframe.addCross(d.pivotB(), 0.08f, color);
                wireframe.addLine(d.pivotA(), d.pivotA().add(d.axisA().normalize(new Vector3f()).mul(0.5f, new Vector3f()), new Vector3f()), color);
            }
        }

        if (cfg.showVelocities()) {
            float[] color = DebugColourPolicy.velocityColour();
            for (Ode4jBodyHandle body : world.debugBodiesInIdOrder()) {
                if (body.mode() != BodyMode.DYNAMIC) {
                    continue;
                }
                BodyState state = world.getBodyState(body);
                Vector3f velocity = state.linearVelocity();
                if (velocity.length() < 0.0001f) {
                    continue;
                }
                Vector3f end = state.position().add(new Vector3f(velocity).mul(0.2f), new Vector3f());
                wireframe.addLine(state.position(), end, color);
            }
        }

        if (cfg.showContacts()) {
            float[] color = DebugColourPolicy.contactColour();
            List<Ode4jContactDispatcher.DebugContact> contacts = world.debugDrainContacts();
            for (Ode4jContactDispatcher.DebugContact c : contacts) {
                wireframe.addCross(c.position(), 0.04f, color);
                Vector3f n = c.normal().normalize(new Vector3f()).mul(0.25f + c.depth() * 0.25f);
                wireframe.addLine(c.position(), c.position().add(n, new Vector3f()), color);
            }
        }

        lastVertexCount = wireframe.vertexCount();
        uploadIfAvailable();
    }

    public void record(VkCommandBuffer commandBuffer) {
        if (staging != null && boundsBuffer != null && commandBuffer != null) {
            staging.flush(commandBuffer);
        }
    }

    public void destroy() {
        if (staging != null) {
            staging.destroy();
        }
        if (boundsBuffer != null) {
            boundsBuffer.destroy();
        }
    }

    public int lastVertexCount() {
        return lastVertexCount;
    }

    public int lastLineCount() {
        return wireframe.lineCount();
    }

    public float[] vertexData() {
        return wireframe.vertices();
    }

    private boolean anyEnabled(PhysicsDebugConfig cfg) {
        return cfg.showShapes()
            || cfg.showContacts()
            || cfg.showConstraints()
            || cfg.showVelocities()
            || cfg.showRaycasts()
            || cfg.showIslands()
            || cfg.showAabbs()
            || cfg.showSleepState();
    }

    private void uploadIfAvailable() {
        if (boundsBuffer == null) {
            return;
        }
        int vertices = wireframe.vertexCount();
        float[] data = wireframe.vertices();
        for (int i = 0; i < vertices; i++) {
            int base = i * 7;
            boundsBuffer.writeBounds(i, data[base], data[base + 1], data[base + 2], 0.01f);
        }
        boundsBuffer.flush();
        if (staging != null) {
            staging.markDirty(boundsBuffer.bufferHandle(), 0L, (long) vertices * Float.BYTES * 7L);
        }
    }
}
