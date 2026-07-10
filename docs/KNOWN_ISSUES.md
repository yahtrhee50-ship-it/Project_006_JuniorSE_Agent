# KNOWN_ISSUES

## Open issues

- **P13 moving-load deliverables still broken (2026-07-09).** `Project_005…\outputs\moving_load_P13.sdb`
  and `moving_load_P13_kipft.sdb` still contain a SAP2000 *library* standard vehicle, which a
  non-Bridge license strips to a flat load with a CSiBridge conversion warning on open. The P005
  builder fix (all trucks as general vehicles, commit `61a6a4f`) is in place, but rebuilding these
  two files requires an **engineer-confirmed P13 axle train** (candidate from library read-back:
  26 kip steer + 6×48 kip @ 18 ft, ~314 kip GVW — confirm vs current Caltrans BDA). Engineer chose
  to defer the rebuild on 2026-07-09. When confirmed, rebuild via `build-from-json` with
  `truck_axle_loads`/`truck_axle_spacings` (the builder refuses `truck_type: "P13"` by policy).
- **Ch 15 P9 textbook discrepancy** (carried): reference file says 44.2 kN·m; solver + 3-moment
  equation both give 57.6 kN·m (wL²/10). Engineer to check the original textbook page.

## Notes

Use this file to record known bugs, limitations, or unfinished items that still matter to the project.