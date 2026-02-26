# Replay Packet Schema (Frozen v1)

Replay packets are JSON and must satisfy the frozen schema contract below.

## Header Fields

- `magic`: `"DYRP"` (constant)
- `formatVersion`: `1`
- `SCHEMA_FINGERPRINT`: `repro-packet-v1` (code constant for change discipline)

## Compatibility Policy

- Current policy is bounded compatibility:
  - `MIN_SUPPORTED_SCHEMA_VERSION = 1`
  - `MAX_SUPPORTED_SCHEMA_VERSION = 1`
  - versions outside this range are rejected with a clear error.
- Invalid `magic` is rejected with a clear error.

## Decode Validation

On decode, the runtime validates:

1. `magic` matches expected constant.
2. `formatVersion` matches expected version.
3. `initialSnapshotB64` is present.

## Design Intent

- Prevent silent replay format drift.
- Fail fast on incompatible packets.
- Keep schema evolution explicit and test-gated.
