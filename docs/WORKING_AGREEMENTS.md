# AI and Engineering Working Agreements

This document is the shared operating contract for human contributors, Claude,
Codex, and other engineering agents working in this repository.

## Source-of-truth order

When sources disagree, use this order:

1. verified code, tests, and captured hardware evidence;
2. accepted architecture decision records or explicitly approved decisions;
3. current status and requirements documents;
4. open issues and pull requests;
5. conversation history.

Conversation history is useful context but is not a durable project record.

## Claim states

Label consequential claims using one of these states:

- **Observed:** directly measured or reproduced; link the artifact or procedure.
- **Inferred:** reasoned from observations; state the reasoning and uncertainty.
- **Proposed:** a candidate awaiting review or a decision gate.
- **Accepted:** approved for implementation or baselining.
- **Rejected/Superseded:** retained for traceability but no longer active.

Vendor specifications are sourced facts, not project verification. Record the
source URL and access date, then bench-verify claims that affect the design.

## Cross-repository ownership

The repository running an experiment owns its procedure, raw outputs, failures,
and conclusions. A consuming repository links that evidence and makes its own
decision through its normal ADR/requirements/BOM process.

Do not copy a conclusion into another repository as an accepted requirement.
Do not change another product's BOM merely because a related prototype worked.

For the current depth work:

- Project Platypus owns Tab5/M024/VL53L8CX experiments and evidence.
- PlatypusOne may list depth sensors as candidates.
- PlatypusOne selects hardware only after its depth-sensing ADR gate passes.

## Branch and work ownership

- Inspect open pull requests before starting work.
- Use one purpose per branch and avoid editing another active agent's branch
  unless the pull request explicitly owns the follow-up.
- State stacked branch dependencies in the pull request body.
- Prefer small documentation or implementation slices that can be reviewed and
  merged independently.
- Never resolve an unknown by silently choosing a value. Record it as a gate,
  assumption, or blocker.

## Evidence records

A hardware conclusion must identify:

- exact hardware and revision;
- wiring, supplies, firmware, and configuration;
- date and environmental conditions;
- procedure and raw artifact locations;
- expected and actual results;
- failures, invalid data, and limitations;
- whether the result is sufficient to promote a downstream decision.

Keep raw evidence immutable. Derived summaries must link back to it.

## Required pull-request handoff

Every agent-authored pull request ends with:

```markdown
## Handoff

- Changed:
- Verified:
- Not verified:
- Decisions made:
- Assumptions:
- Blockers:
- Cross-repository effects:
- Next safe action:
```

“Verified” must name the command, test, inspection, or hardware procedure.
Research-only work says what was source-verified and what still requires a
bench test.
