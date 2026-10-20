# ARC IEEE paper (conference format, IEEEtran)

- `arc_ieee.tex`: the paper. On Overleaf, upload this zip; it compiles with pdfLaTeX, no extra packages. Text in red `[TO UPDATE: ...]` marks numbers still to come: A4 pairs 15–19, phase-2 results, author details.
- `figs/`: figures as vector PDF (used by LaTeX) and 300-dpi PNG.
- `make_figs.py`: redraws every figure from the logs in this folder:
  - `c2_log.txt`: the C2 terminal log;
  - `b1_trace.txt`: the B1 sample trace;
  - the A1 and C1 numbers come from results.md (16 Oct 09:20).

  Rerun it after new results.
- References: check every entry against the original before submission.
