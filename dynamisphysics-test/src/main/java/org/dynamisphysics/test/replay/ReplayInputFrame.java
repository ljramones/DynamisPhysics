package org.dynamisphysics.test.replay;

import java.util.List;

public record ReplayInputFrame(int step, List<ReplayOp> ops) {
    public ReplayInputFrame {
        ops = ops == null ? List.of() : List.copyOf(ops);
    }
}
