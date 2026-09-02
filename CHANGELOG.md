# Changelog

---

<details markdown="1">
  <summary>Table of Contents</summary>

<!-- TOC -->
* [Changelog](#changelog)
  * [v0.1.0 (2026-07-14)](#v010--2026-07-14-)
  * [v0.1.1 (2026-07-15)](#v011--2026-07-15-)
  * [v0.2.0 (2026-09-01)](#v020--2026-09-01-)
<!-- TOC -->

</details>

---

## [v0.1.0 (2026-07-14)](https://github.com/scalpelspace/scalpelspace_bus/releases/tag/v0.1.0)

- Initial release.

---

## [v0.1.1 (2026-07-15)](https://github.com/scalpelspace/scalpelspace_bus/releases/tag/v0.1.1)

- Update `mc_brushed_driver` vendor files for v0.1.1 release.

---

## [v0.2.0 (2026-09-01)](https://github.com/scalpelspace/scalpelspace_bus/releases/tag/v0.2.0)

- Fix minor docs formatting and styling issues in `README.md`.
- Update vendored sources: `can_driver` v0.6.0, `momentum_driver` v0.5.0,
  `mc_stepper_driver` v0.3.0 and `mc_brushed_driver` v0.2.0.
- Support `can_driver` v0.6.0 node ID allocation:
    - Adopt the new `node_id_assignment_ctx_t` assignment strategy signature.
    - Accept `ADVERTISE` from nodes that already hold a node ID (the full
      `0x720`..`0x73E` CAN ID range) and decode the new `alloc_mode` field.
    - Honour nodes with a fixed (hardcoded) node ID: their node ID is taken out
      of the assignable pool and they are never sent an assignment.
- Add `ScalpelBusNode::reserved`, set for nodes holding a fixed node ID.
