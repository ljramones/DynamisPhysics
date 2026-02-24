# Repository Guidelines

## Project Structure & Module Organization
This repository is currently a minimal scaffold. Keep the layout predictable as code is added:
- `docs/`: design notes, architecture decisions, and contributor-facing documentation.
- `src/main/java/`: production Java code (create as needed).
- `src/test/java/`: automated tests mirroring `src/main/java` package structure.
- `assets/` (optional): static diagrams, sample data, or non-code resources.

Example package path: `src/main/java/com/dynamisphysics/simulation/`.

## Build, Test, and Development Commands
The repo does not yet include a build wrapper. When initializing, prefer Maven with Java 25 (`.java-version`):
- `mvn -q clean compile`: compile main sources.
- `mvn -q test`: run unit/integration tests.
- `mvn -q spotless:apply`: apply formatting (if Spotless is configured).
- `mvn -q verify`: full local pre-PR validation.

If Gradle is adopted instead, document equivalent `./gradlew` commands in this file.

## Coding Style & Naming Conventions
- Use 4-space indentation; no tabs.
- Follow standard Java conventions: `PascalCase` for classes, `camelCase` for methods/fields, `UPPER_SNAKE_CASE` for constants.
- Keep packages lowercase and domain-based (for example `com.dynamisphysics.core`).
- Prefer small, focused classes; avoid wildcard imports.
- Run formatter/linter before committing.

## Testing Guidelines
- Use JUnit 5 for unit tests.
- Test class naming: `<ClassName>Test`; integration tests: `<FeatureName>IT`.
- Mirror production package paths in `src/test/java`.
- Add tests for every bug fix and new behavior.
- Target meaningful coverage on changed code; avoid untested logic branches.

## Commit & Pull Request Guidelines
No commit history exists yet; adopt Conventional Commits now:
- `feat: add rigid body integrator`
- `fix: handle zero-mass collision case`
- `docs: document solver assumptions`

PRs should include:
- clear summary of what changed and why,
- linked issue/ticket (if available),
- test evidence (`mvn -q test` output or equivalent),
- updated docs when behavior or architecture changes.

## Security & Configuration Tips
- Do not commit secrets, credentials, or machine-local configs.
- Keep environment-specific settings in ignored local files.
- Review dependency versions regularly for security updates.
