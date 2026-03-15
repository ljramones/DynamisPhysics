package org.dynamisengine.physics.jolt;

import com.github.stephengold.joltjni.Jolt;

import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.util.ArrayList;
import java.util.List;

/**
 * Handles Jolt native library loading and one-time runtime initialization.
 */
final class JoltNativeLoader {

    private static volatile boolean nativeLoaded;
    private static volatile boolean runtimeInitialized;

    private JoltNativeLoader() {
    }

    static void ensureRuntimeInitialized() {
        if (runtimeInitialized) {
            return;
        }
        synchronized (JoltNativeLoader.class) {
            if (runtimeInitialized) {
                return;
            }
            ensureNativeLoaded();
            Jolt.registerDefaultAllocator();
            Jolt.newFactory();
            Jolt.registerTypes();
            runtimeInitialized = true;
        }
    }

    private static void ensureNativeLoaded() {
        if (nativeLoaded) {
            return;
        }
        synchronized (JoltNativeLoader.class) {
            if (nativeLoaded) {
                return;
            }
            String[] resourceCandidates = {
                "osx/aarch64/com/github/stephengold/libjoltjni.dylib",
                "linux/x86_64/com/github/stephengold/libjoltjni.so",
                "windows/x86_64/com/github/stephengold/joltjni.dll"
            };
            List<String> loadErrors = new ArrayList<>();
            for (String candidate : resourceCandidates) {
                String error = tryLoadFromResource(candidate);
                if (error == null) {
                    nativeLoaded = true;
                    return;
                }
                loadErrors.add(candidate + " -> " + error);
            }
            throw new IllegalStateException(
                "Unable to load jolt-jni native library from bundled resources. "
                    + "Checked: " + String.join("; ", loadErrors)
            );
        }
    }

    private static String tryLoadFromResource(String resourcePath) {
        ClassLoader loader = JoltNativeLoader.class.getClassLoader();
        try (InputStream in = loader.getResourceAsStream(resourcePath)) {
            if (in == null) {
                return "resource-not-found";
            }
            String suffix = resourcePath.endsWith(".dll") ? ".dll"
                : resourcePath.endsWith(".so") ? ".so" : ".dylib";
            Path temp = Files.createTempFile("joltjni-", suffix);
            temp.toFile().deleteOnExit();
            Files.copy(in, temp, StandardCopyOption.REPLACE_EXISTING);
            System.load(temp.toAbsolutePath().toString());
            return null;
        } catch (IOException | UnsatisfiedLinkError ex) {
            return ex.getClass().getSimpleName() + ": " + ex.getMessage();
        }
    }
}
