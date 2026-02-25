package org.dynamisphysics.jolt.vehicle;

import com.github.stephengold.joltjni.Vec3;
import com.github.stephengold.joltjni.VehicleConstraintSettings;
import com.github.stephengold.joltjni.VehicleDifferentialSettings;
import com.github.stephengold.joltjni.VehicleEngineSettings;
import com.github.stephengold.joltjni.VehicleTransmissionSettings;
import com.github.stephengold.joltjni.WheelSettingsWv;
import com.github.stephengold.joltjni.WheeledVehicleControllerSettings;
import com.github.stephengold.joltjni.enumerate.ETransmissionMode;
import org.dynamisphysics.api.VehicleDescriptor;
import org.dynamisphysics.api.WheelConfig;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

final class JoltVehicleAdapter {
    private JoltVehicleAdapter() {
    }

    static VehicleConstraintSettings toConstraintSettings(VehicleDescriptor descriptor) {
        VehicleConstraintSettings settings = new VehicleConstraintSettings();
        settings.setUp(new Vec3(0f, 1f, 0f));
        settings.setForward(new Vec3(0f, 0f, -1f));

        List<WheelSettingsWv> wheelSettings = new ArrayList<>(descriptor.wheels().size());
        for (WheelConfig wheel : descriptor.wheels()) {
            WheelSettingsWv ws = new WheelSettingsWv();
            ws.setPosition(new Vec3(
                wheel.attachmentPoint().x(),
                wheel.attachmentPoint().y(),
                wheel.attachmentPoint().z()
            ));
            ws.setWheelUp(new Vec3(0f, 1f, 0f));
            ws.setWheelForward(new Vec3(0f, 0f, -1f));
            ws.setSteeringAxis(new Vec3(0f, 1f, 0f));
            ws.setSuspensionDirection(new Vec3(0f, -1f, 0f));
            ws.setRadius(wheel.radius());
            ws.setWidth(wheel.width());
            ws.setSuspensionMinLength(0f);
            ws.setSuspensionMaxLength(wheel.suspensionTravel());
            ws.setSuspensionPreloadLength(wheel.suspensionTravel() * 0.5f);
            ws.getSuspensionSpring().setStiffness(wheel.springStiffness());
            ws.getSuspensionSpring().setDamping(wheel.damping());
            ws.setMaxSteerAngle(wheel.steered() ? (float) java.lang.Math.toRadians(30f) : 0f);
            ws.setMaxBrakeTorque(2_500f);
            ws.setMaxHandBrakeTorque(wheel.steered() ? 0f : 4_000f);
            wheelSettings.add(ws);
        }
        settings.addWheels(wheelSettings.toArray(WheelSettingsWv[]::new));

        WheeledVehicleControllerSettings controller = new WheeledVehicleControllerSettings();
        configureEngine(controller.getEngine(), descriptor);
        configureTransmission(controller.getTransmission(), descriptor);
        configureDifferentials(controller, descriptor);
        settings.setController(controller);
        return settings;
    }

    private static void configureEngine(VehicleEngineSettings engine, VehicleDescriptor descriptor) {
        engine.setMaxTorque(descriptor.engine().maxTorqueNm());
        engine.setMinRpm(descriptor.engine().idleRpm());
        engine.setMaxRpm(descriptor.engine().maxRpm());
        engine.setAngularDamping(org.vectrix.core.Math.max(0f, descriptor.engine().engineBrakeTorque()) * 0.1f);
    }

    private static void configureTransmission(VehicleTransmissionSettings transmission, VehicleDescriptor descriptor) {
        transmission.setMode(descriptor.transmission().automatic() ? ETransmissionMode.Auto : ETransmissionMode.Manual);
        transmission.setGearRatios(descriptor.transmission().gearRatios());
        transmission.setReverseGearRatios(org.vectrix.core.Math.abs(descriptor.transmission().reverseRatio()));
        transmission.setClutchStrength(descriptor.transmission().clutchStrength());
        transmission.setShiftUpRpm(descriptor.transmission().shiftUpRpm());
        transmission.setShiftDownRpm(descriptor.transmission().shiftDownRpm());
    }

    private static void configureDifferentials(WheeledVehicleControllerSettings controller, VehicleDescriptor descriptor) {
        List<Integer> driven = new ArrayList<>();
        for (int i = 0; i < descriptor.wheels().size(); i++) {
            if (descriptor.wheels().get(i).driven()) {
                driven.add(i);
            }
        }
        if (driven.size() < 2) {
            return;
        }
        driven.sort(Comparator.naturalOrder());
        controller.setNumDifferentials(1);
        VehicleDifferentialSettings diff = controller.getDifferential(0);
        diff.setLeftWheel(driven.get(0));
        diff.setRightWheel(driven.get(1));
        diff.setDifferentialRatio(descriptor.transmission().finalDriveRatio());
        diff.setLimitedSlipRatio(org.vectrix.core.Math.max(0f, descriptor.differential().limitedSlipBias()));
        diff.setEngineTorqueRatio(1f);
    }
}
