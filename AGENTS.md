# Repository guidance for coding agents

These reusable profiles apply to the repository containing this file. Select
the relevant profile for the requested work and adapt it to that project's
hardware, framework version, architecture, and requirements. No particular
application, source-file layout, sensor, actuator, or loop rate is assumed.

## Predictable acquisition/control loops on ESP32-S3

Apply this guidance when creating or changing a timing-sensitive acquisition or
control path, its drivers, or services that can interfere with it. For unrelated
work, do not refactor the application to satisfy this section. These are design
and verification rules, not a guarantee of hard real-time behavior. Recheck chip
capabilities and ESP-IDF version when reusing them in another project; not every
ESP32-family device has two cores or the same peripherals/configuration symbols.

### Timing contract and execution model

1. Define the acquisition/control rates, end-to-end deadline, allowable jitter,
   and response to late events, stale samples, and peripheral failures before
   choosing optimizations. Budget interrupt latency, task wake-up, acquisition,
   computation, actuation, and occasional housekeeping with execution headroom.
2. For tight timing, prefer a hardware timing source such as GPTimer or a
   peripheral DMA event. Use absolute deadlines or hardware auto-reload to avoid
   accumulating callback latency. Document timer quantization and any fractional
   period pattern. A DMA frame-completion event represents a batch, not
   necessarily one sample; distinguish sampling time from notification time.
3. Keep ISRs bounded and short: acknowledge/rearm as required, capture minimal
   state, and notify the control task using ISR-safe APIs. Request a context
   switch according to the driver's callback contract. Run sensor processing,
   floating-point control, and ordinary driver APIs in task context. Initialize
   task handles, buffers, and callback state before enabling the interrupt source.
4. A notification-driven task may block between releases with
   `ulTaskNotifyTake()`. Explicitly handle its returned count: `pdTRUE` clears all
   accumulated notifications, so one iteration may represent multiple releases.
   Count overruns and define whether to discard, process, or fault on backlog;
   do not silently run unlimited catch-up iterations. Define recovery after a
   delayed ISR or an alarm deadline that has already passed.
5. On dual-core ESP32-S3, prefer explicit task affinity for the critical owner
   task (normally Core 1), with logging, USB/network work, and housekeeping on
   Core 0. Choose priority from the task/driver dependencies and timing budget;
   maximum priority is not automatically necessary. Audit system and library
   task affinities, using supported configuration before patching dependencies.
6. Audit interrupt affinity separately from task affinity. Install/enable a
   driver from the intended pinned task where its allocation semantics require
   this. Pinning `app_main` does not pin every task or interrupt it creates.
   Core affinity reduces competition but does not eliminate tick interrupts,
   cross-core interrupts, shared-memory contention, or shared spinlocks.

### Bound work, resource access, and memory

7. Give the critical task exclusive ownership of its peripheral/state when
   practical. For an SPI sensor or actuator on a dedicated available
   general-purpose host, prefer calling `spi_device_acquire_bus()` once before
   the periodic loop and retaining ownership across iterations for the entire
   continuous operating phase. This avoids repeated bus acquisition/release
   overhead and contention with other devices; the ESP-IDF documentation
   explicitly recommends retaining ownership across polling transactions to
   avoid the per-transaction task-unblocking overhead. Do not release/reacquire
   on every sample when exclusive ownership can be retained. Release on shutdown
   or error paths that end that operating phase, after completing outstanding
   transactions. Treat permanent reservation as an opt-in operating mode, not a
   universal component constraint: a reusable component should default to
   shared-bus-compatible behavior, support attaching to an application-owned
   bus, and let each instance acquire/release exclusivity around a continuous
   high-rate phase. Do not make a global Kconfig choice the only way to select
   this lifecycle policy, because instances in one firmware may differ. The
   reservation belongs to one device handle: if the loop must access multiple
   SPI devices on the same host, design their arbitration rather than
   permanently reserving the host for only one of them. Bus ownership does not
   prevent task preemption. Apply the ownership principle to I2C and other
   resources without copying SPI-specific APIs.
8. Bound peripheral waits, retries, lock hold times, and per-cycle work. An
   infinite wait for the next release can be intentional; an infinite sensor
   transaction cannot establish a bounded cycle time. Account for driver timeout
   resolution and failure paths. Remove locks only after proving exclusive
   ownership. For cross-core data use coherent snapshots, bounded queues, or
   appropriate atomics; `volatile` alone does not provide synchronization.
9. Allocate and initialize resources before starting periodic work. Keep dynamic
   allocation, formatted logging, file/NVS operations, and blocking USB/network
   transfers outside the critical path. Publish bounded snapshots to a worker
   with an explicit overflow/drop policy. Include snapshot/copy overhead in the
   timing budget even when it occurs only once per reporting window.
10. Use internal memory placement when the latency budget or cache-off execution
    requirement justifies it. `IRAM_ATTR` places a function, not its complete
    call graph; `static` controls linkage/storage duration, not IRAM placement.
    Audit callees, compiler helpers, constants, buffers, stack, and driver/RTOS
    configuration as applicable. Verify the final ELF/map rather than assuming
    an annotation makes the whole path cache-safe. Account for internal RAM use.
11. Distinguish an ISR that can run with flash cache disabled from a task that
    can continue meeting deadlines then. IRAM alone does not make a suspended
    scheduler run. Defer flash writes/erases until the loop is stopped when
    necessary; otherwise verify the complete supported execution path and its
    timing under those operations. Do not claim flash/cache immunity from a
    timer configuration alone.
12. Prefer supported driver APIs. Introduce HAL/LL/register access only after
    measuring a relevant bottleneck and checking register semantics, pin
    validity, ownership, concurrency, and ESP-IDF compatibility. Do not assume
    every GPIO API takes a spinlock: in ESP-IDF v6.0, `gpio_set_level()` validates
    the output pin and calls the HAL without a spinlock in that function.

### Arithmetic, configuration, and validation

13. Prefer `float`, `f`-suffixed floating constants, and float math functions when
    single precision meets numerical requirements; ESP32-S3 does not accelerate
    `double` in hardware. Precompute invariant work. Replacing division by a
    constant with multiplication by its reciprocal is optional: check rounding,
    numerical stability, generated code, and measured benefit. Do not promise
    fixed cycle counts for C expressions or enable unsafe math transformations
    without validating control/estimator behavior.
14. Keep floating-point work outside ISRs by default and leave
    `CONFIG_FREERTOS_FPU_IN_ISR` disabled when no ISR needs it. An integer-only
    ISR does not corrupt a preempted task merely because that task uses the FPU.
    Check the port's restrictions before enabling ISR floating point.
15. Preserve interrupt/task watchdog protection. Blocking between releases only
    helps if sufficient CPU time remains for monitored tasks (including Idle).
    Verify this under sustained load and backlog; do not disable watchdogs to
    hide starvation. Audit CPU/peripheral clocks, dynamic frequency scaling,
    and sleep; configure supported power-management locks where needed.
16. Make relevant defaults reproducible in `sdkconfig.defaults` without replacing
    unrelated settings. Verify symbols against the selected ESP-IDF Kconfig and
    inspect the effective `sdkconfig`; defaults do not override existing choices.
    Check main-task, ESP Timer task/ISR, FreeRTOS timer-service, and enabled
    network task affinities separately. ESP Timer is not the FreeRTOS tick or
    its software-timer service. Set only options supported and needed by the app.
17. For implementation changes, build and run relevant tests, then measure on
    hardware under representative competing load and peripheral faults. Record
    maximum observed execution time, release-to-ISR/task latency, sample/actuator
    intervals, deadline misses, backlog/drops, and measurement overhead. Include
    occasional reporting work and test production settings. Use external timing
    capture when needed: an ISR-entry timestamp alone cannot reveal how late
    that ISR was relative to its hardware deadline. Observed maxima are not a
    proven worst-case bound; clearly report when hardware validation is pending.

Before changing an existing application, locate its timing source, critical
task, peripheral drivers, background workers, and effective build configuration.
Evaluate them against the project's timing contract; do not assume that existing
code already satisfies these rules. For a new application, choose loop rates,
peripherals, and source organization from its requirements.

### Separation of repository-wide and application-specific guidance

Keep this `AGENTS.md` reusable across applications in the repository. Do not
add a sensor name, application path, pin assignment, estimator, task name,
fixed loop rate, payload schema, or application-specific migration plan here.
When a particular example or firmware needs such instructions, create or
update a sibling `AGENTS-APP.md` in that application's directory (or at the
repository root only when it genuinely applies to every application). An
`AGENTS-APP.md` may specialize these rules but may not weaken the timing,
ownership, safety, or validation requirements above. Agents changing an
application must read its applicable `AGENTS-APP.md`; agents changing a
reusable component must not copy application details into this file.

Technical references (ESP-IDF v6.0, ESP32-S3; verify for the selected version):

- [FreeRTOS affinity, critical sections, and FPU behavior](https://docs.espressif.com/projects/esp-idf/en/v6.0/esp32s3/api-reference/system/freertos_idf.html)
- [SPI bus ownership and transaction costs](https://docs.espressif.com/projects/esp-idf/en/v6.0/esp32s3/api-reference/peripherals/spi_master.html)
- [Execution speed and memory placement](https://docs.espressif.com/projects/esp-idf/en/v6.0/esp32s3/api-guides/performance/speed.html)

## Optional real-time diagnostics components

Use this profile when an application needs evidence about release delivery,
wake latency, cycle execution time, stage budgets, interval jitter, or deadline
overruns. The reusable components are:

- `esp_rt_diagnostics`
  ([repository](https://github.com/smartsensingme/esp_rt_diagnostics)) is the
  bounded accumulator. One `esp_rt_diag_t` belongs to one monitored task; it
  records numeric counters and creates value-owned snapshots. It does not
  create the timer, log, allocate in the hot path, format text, or interpret
  sensor/application state.
- `esp_rt_diagnostics_reporter`
  ([repository](https://github.com/smartsensingme/esp_rt_diagnostics_reporter))
  is the asynchronous presentation companion. It copies one completed
  snapshot and an optional fixed-size application payload into a one-slot
  overwrite queue, then formats or calls back from a lower-priority task. It
  must not be called from an ISR and it is not a lossless recorder.

For a Component Manager consumer, list only
`esp_rt_diagnostics_reporter` in the consuming application's
`main/idf_component.yml`; its manifest resolves the compatible transitive
`esp_rt_diagnostics` revision. Keep `CMakeLists.txt` dependency declarations
and the manifest's acquisition/pinning roles distinct. For an offline/local
experiment, use explicit local component directories or an approved Component
Manager override and document the selected revisions; never silently depend on
a moving branch in a released integration.

When using these components, read their current `AGENTS.md`, public headers,
and README files before changing the integration. Preserve the following
workflow:

1. Define the measured cycle first: identify the exact release event and rate,
   deadline, work executed once per release, and any divided-rate work. The
   diagnostic `expected_period_us` describes the event that calls
   `esp_rt_diag_cycle_begin*`, not an arbitrary sensor sub-operation.
2. Allocate contiguous numeric IDs for actionable stages, events, and
   intervals. Initialize the accumulator before enabling the periodic source.
   Configure a window long enough to contain many cycles and keep counters from
   overflowing. A five-second starting window is useful for kHz development,
   but is not a universal requirement.
3. In the release ISR, call only `esp_rt_diag_isr_capture()` near the task
   notification. In the owner task, pass the complete notification count to
   `esp_rt_diag_cycle_begin_from_isr()`. More than one pending event is evidence
   of accumulated releases; do not treat it as a Boolean or invent wake latency
   for every event.
4. Record bounded stages around variable-duration work, events for occurrences
   and faults, and intervals whose timestamp endpoints are explicitly defined.
   Pair every successful cycle begin with exactly one cycle end, including
   sensor-error and recovery paths. Take a snapshot only after ending the
   cycle. Do not log, format, allocate, block on a queue, or publish through
   the reporter from the monitored task's critical section.
5. Copy/publish the completed snapshot after the cycle boundary. Configure the
   reporter below the monitored task's priority, preferably on the other core,
   and start with compact output or a bounded callback. Its one-slot overwrite
   behavior is intentional; a skipped `window_id` means presentation could not
   keep up. Keep stage/event/interval name arrays and callback context alive for
   the complete reporter lifetime.
6. Keep application-specific values in the reporter payload/callback, not in
   generic diagnostic structures. Payload copying is bounded but contributes to
   the producer cost, so include it in the timing budget or close the measured
   cycle before the handoff and document that choice. Interpret evidence from
   repeated quiet windows and log-affected windows separately; report counts
   with denominators rather than treating one maximum as a worst-case proof.

Do not use these components to replace an application's sensor validity policy,
stale-event handling, estimator diagnostics, or data protocol. They measure
and present evidence; the application remains responsible for deciding what a
sample means and for choosing a separate recorder/stream when every sample
must be retained. Put the concrete mapping for a specific application in its
`AGENTS-APP.md`, not here.

## Comprehensive component documentation profile (opt-in)

This profile defines a detailed documentation style intended for components
that are being finalized for teaching, reuse, or release. **Do not apply this
profile automatically.** Apply it only when the user explicitly requests
comprehensive/final component documentation, asks for the established
documentation pattern, or explicitly invokes this profile.

For ordinary implementation, diagnosis, bug fixes, and refactoring, update only
the documentation directly affected by the behavior change. Do not expand
unrelated comments or READMEs merely to match this profile. This opt-in rule
keeps routine changes focused and avoids unnecessary review effort and token
consumption.

When the profile is explicitly requested:

1. Read the component's source, public headers, README, Kconfig, build files,
   examples, and directly coupled application integration before documenting.
   Describe the code that actually exists; do not infer an obsolete design.
2. In public headers, document every public function, type, enum, field, macro,
   and callback. State purpose, units, valid ranges, ownership and lifetime,
   preconditions, state changes, concurrency/real-time constraints, parameters,
   return values, and relevant failure modes.
3. For every function, say whether it is called externally, internally, or
   both. When internal functions call it, name those callers. When understanding
   the implementation benefits from it, identify important functions it calls.
4. In source files, document every public and private function. Divide each
   implementation into functional blocks with comments explaining how and why
   the block works. Add local comments where a non-obvious invariant,
   conversion, synchronization rule, saturation rule, or protocol decision
   matters. Avoid comments that merely restate C syntax.
5. Make the component README sufficient for a new user to integrate it. Cover
   purpose and boundaries, architecture, lifecycle/state machine, configuration
   and Kconfig fields, complete usage examples, API behavior, data formats,
   units, memory ownership, concurrency, deterministic-loop considerations,
   error handling, and troubleshooting as applicable.
6. For wire protocols or stored formats, document framing, byte order, layout,
   scaling/reconstruction, reserved values, integrity checks, retry behavior,
   and compatibility/version rules.
7. Improve Kconfig help so each option explains behavior, resource cost, and
   important interactions instead of only repeating its prompt.
8. If APIs were intentionally removed, summarize that fact in a compatibility
   or removal log; do not preserve obsolete functions solely for documentation.
9. Preserve the language and file set already used by the component. Do not add
   translated or duplicate documentation unless the user requests it or the
   component already maintains those documents in parallel.
10. Finish with formatting, relevant builds/tests, and `git diff --check`.
    Review the resulting documentation against the current implementation and
    correct statements that are broader than the code guarantees.

User instructions for a particular task override this profile. The profile
governs documentation depth, not permission to change behavior or public APIs.

## Independent ESP-IDF component extraction profile (opt-in)

Apply this profile only when the user explicitly asks to transform a component
of this project into an external or independent component. Do not externalize
components automatically during ordinary refactoring.

When this profile is requested:

1. Audit the component boundary before extracting it. Identify its public API,
   source files, Kconfig options, tests, examples, documentation, ESP-IDF
   dependencies, and dependencies on other project components. Move
   application-specific behavior out of the component when that can be done
   without weakening its generality.
2. Make the extracted repository usable at its root as an ESP-IDF component.
   It should contain its own `CMakeLists.txt`, public headers, source files,
   Kconfig when applicable, README, license information, and directly relevant
   tests or examples. Do not require the consumer to reproduce files from this
   application's `main/` directory.
3. Preserve both kinds of dependency declaration:
   - use `REQUIRES` or `PRIV_REQUIRES` in the component's `CMakeLists.txt` to
     describe the ESP-IDF compile/link relationship;
   - create `idf_component.yml` in the root of the independent component to
     tell the ESP-IDF Component Manager how to obtain every external component
     dependency.
4. For a Git dependency in the component's `idf_component.yml`, use a portable
   public URL such as `https://github.com/OWNER/REPOSITORY.git`. Pin a tested
   tag or full commit hash instead of relying silently on a moving branch.
   Declare the supported ESP-IDF version explicitly. Never put a developer's
   SSH host alias, filesystem path, credential, or token in a tracked manifest.
5. Document the consumer-side installation using the application's
   `main/idf_component.yml`. The application should list only the component it
   directly consumes; the installed component's root `idf_component.yml` must
   resolve its transitive dependencies automatically. Make this distinction
   explicit: `main/idf_component.yml` belongs to the consuming application,
   while the component's `idf_component.yml` belongs to the reusable
   component.
6. Keep a manual/offline installation path documented when practical. If the
   Component Manager would otherwise select `managed_components/`, explain how
   to disable it with `IDF_COMPONENT_MANAGER=0` or how to use an explicit local
   override. Do not claim that a sibling local component automatically wins
   unless that behavior was verified with the supported ESP-IDF version.
7. Preserve useful Git history when extracting the component. Create and
   verify the independent component commit before changing the parent project.
   Then replace the in-tree directory with a Git submodule whose tracked URL in
   `.gitmodules` is the standard public HTTPS GitHub URL.
8. Keep authentication configuration local. A developer-specific SSH alias may
   be configured as `remote.origin.pushurl` to select the correct publishing
   key, while `remote.origin.url` and every tracked file remain portable HTTPS
   URLs. Never commit the alias. Do not push unless the user explicitly asks;
   when the user intends to run credentialed commands, provide the exact push
   commands and the required order.
9. Publish dependencies before dependents, and publish the independent
   component before the parent commit that points its submodule at the new
   revision. This prevents the parent repository or a transitive manifest from
   temporarily referencing commits that cannot be fetched.
10. Verify the result with `git diff --check`, a component-relevant test, and an
    ESP-IDF configure/build. For components with transitive dependencies, also
    verify that the Component Manager resolves them from a consumer that lists
    only the top-level component. Confirm that generated
    `managed_components/` content is not committed and decide deliberately
    whether the consuming application should commit `dependencies.lock`.
11. Report the independent-component commit and the parent integration commit
    separately. Also report unpushed commits, required publication order, and
    any unrelated working-tree changes that were intentionally left untouched.
