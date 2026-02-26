# Replay Backend Capability Matrix

| Backend | STRICT Replay | BEHAVIOURAL Replay | Long Determinism Soak | Canonical Reference Required | Notes |
|---|---:|---:|---:|---:|---|
| ODE4J | ✅ | ✅ | ✅ | ✅ | STRICT passes under fresh-restore canonical reference. |
| Jolt | ✅ | ✅ (PERF) | ✅ (gated) | ✅ | Same-world post-build history can diverge from fresh-restore history; diagnostic only. |

## Global Replay Contracts

- Snapshot-first is canonical for replay validation.
- STRICT references must be generated from canonical history:
  `new world -> apply tuning -> restore(snapshot) -> step -> hash`.
- Same-world post-build history equivalence is diagnostic-only and must not be used for STRICT references.
- STRICT determinism prerequisites:
  - fixed timestep
  - deterministic profile/tuning
  - single-thread stepping where required by backend
  - pinned allocator mode where required by backend
