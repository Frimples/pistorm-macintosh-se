# PiStorm Macintosh SE Test Bugs

This tracker belongs to the isolated test repository. It does not describe or modify the known-good production repository.

## Status values

- **Open** — reproduced or supported by current evidence; no verified fix.
- **Investigating** — active diagnosis or instrumentation.
- **Candidate** — a proposed fix exists but hardware verification is incomplete.
- **Verified** — reproduced as fixed on the Macintosh SE hardware.
- **Closed** — no longer relevant or superseded by a tracked issue.

## Bugs

### BUG-001 — Physical Macintosh SE reset freezes emulation

- **Status:** Open
- **Area:** Emulator reset path
- **First observed with:** System 7.1.1 physical reset button
- **Reproduction:** Boot System 7.1.1, press the physical Macintosh SE reset button.
- **Observed:** The emulator logs:
  ```text
  Amiga Reset is down...
  Amiga Reset is up...
  ```
  but the emulated CPU does not resume normally and the Macintosh freezes.
- **Control:** System 7.1.1 software restart works.
- **Evidence:** The IPL thread detects the external reset edge. It sets `do_reset`; the CPU thread is expected to consume that request and call `cpu_pulse_reset()`. The current path uses a fixed delay and does not explicitly hold CPU execution until the reset line is deasserted.
- **Scope:** Fix the generic hardware-reset path before using physical-reset behavior to evaluate SCSI changes.
- **Not a verified cause:** The SCSI add-on has not been proven to cause this issue.

### BUG-002 — SCSI target requires delayed activation during cold boot

- **Status:** Open
- **Area:** `platforms/macse` virtual SCSI
- **First observed with:** System 6.0.8 / `scsi_image` enabled at startup
- **Reproduction:** Configure `platform macse` and `scsi_image`, then cold-boot the Macintosh SE.
- **Observed:** The machine does not boot normally unless the SCSI add-on is first disabled, the system is allowed to reach the flashing-floppy screen, and the emulator is then restarted with SCSI enabled.
- **Current workaround:** Delay SCSI ownership of `$580000–$5FFFFF` until the first access to the Macintosh SE IWM/floppy window `$D00000–$DFFFFF`.
- **Current evidence:** MAME shows the SE ROM accesses the SCSI window early and initializes mirrored NCR 5380 registers before continuing through floppy initialization.
- **Candidate behavior:** The delayed-activation binary reaches normal boot operation according to hardware testing, but the complete cold-boot path is not yet marked verified.
- **Scope:** Keep investigation limited to SCSI activation timing, NCR 5380 register semantics, and pseudo-DMA behavior.

### BUG-003 — Macintosh restart behavior with virtual SCSI is unresolved

- **Status:** Open
- **Area:** Guest restart / virtual SCSI state
- **First observed with:** System 6.0.8 and SCSI add-on enabled
- **Reproduction:** Boot with the virtual SCSI image active, then select Restart from the Macintosh command bar.
- **Observed:** The Macintosh SE resets, but the emulator only reports the external reset transition and does not complete a usable emulated restart.
- **Controls:** System 7.1.1 software restart works; the physical reset button freezes System 7.1.1 as tracked in BUG-001.
- **Scope:** Track separately from the cold-boot SCSI activation issue and the generic physical-reset issue.

## New bug template

### BUG-NNN — Short title

- **Status:** Open
- **Area:**
- **First observed with:**
- **Reproduction:**
- **Observed:**
- **Expected:**
- **Control:**
- **Evidence:**
- **Scope:**
- **Candidate fix:**
- **Verification:**
