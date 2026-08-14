## Scope

- [ ] This PR addresses one defined roadmap phase or clearly scoped issue.
- [ ] Unrelated refactoring was avoided.

## Verification

- [ ] The underlying issue was checked against the current source.
- [ ] Relevant automated tests were added or updated.
- [ ] Relevant tests were run.
- [ ] Exact test commands/results are documented.
- [ ] Hardware validation requirements are identified.
- [ ] Hardware validation is not claimed unless actually performed.

## Architecture

- [ ] The change does not create an unintended additional final `/cmd_vel` owner.
- [ ] Alternative/experimental modes remain clearly separated.
- [ ] No missing component is described as implemented.
- [ ] No unknown hardware value was guessed.

## Safety

- [ ] Safety/failure behavior was considered.
- [ ] Watchdogs or stop paths were not weakened.
- [ ] Any change affecting physical motion has explicit validation requirements.

## Documentation

- [ ] Relevant architecture documentation was updated.
- [ ] Relevant ADRs were updated or added.
- [ ] `docs/verification/test_matrix.md` was updated.
- [ ] `docs/verification/evidence_log.md` was updated where evidence was generated.

## Review

- [ ] Remaining risks are listed.
- [ ] Required human/hardware tests are listed.

## Evidence and results

<!-- List exact commands, results, commit/configuration, evidence IDs, and artifact paths. State what could not be tested and why. -->

## Remaining risks and required hardware work

<!-- Preserve evidence labels such as Potential issue; do not imply unperformed hardware tests passed. -->
