# ToF Evidence

This directory stores immutable hardware observations from Project Platypus
depth-sensing experiments.

Create one dated directory per run using the structure defined in
[`TOF_TEST_PLAN.md`](../../TOF_TEST_PLAN.md). Raw measurements and photographs
must not be replaced by cleaned versions. Put plots, filtered exports, and
interpretation under the run's `derived/` directory.

No file in this directory changes the PlatypusOne BOM automatically. A
downstream product decision must link the relevant evidence and pass its own
review gate.
