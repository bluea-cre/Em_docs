# Copilot guide: Deep-dive, mechanisms, and cross-links (ARM Cortex-M4 + STM32F407VET6)

Purpose: Help you explain the why and how, link topics across documents, and give precise, register-level guidance with best practices and pitfalls.

## How to navigate this repo (what to read when)
- `ARM/`
  - TRM: Cortex‑M4 core behavior (exceptions, stacks, lazy stacking)
  - ARMv7‑M Arch Ref: instruction set, memory model, barrier ops
  - Errata notices: silicon/core caveats to factor into design
- `STM32F407VET6/`
  - DS8626 (datasheet): electrical limits, clocks, memory sizes
  - RM0090 (reference manual): all peripheral registers/bitfields
  - PM0214 (Cortex‑M4 programming manual): NVIC, faults, MPU
  - ES0182 (errata): STM32F407/417 known issues and workarounds
  - Board images: quick pinout/footprint reference
- `STM32F407VET6/AN/` and `AN-Tool&Software/`: implementation patterns; prefer these for “how to do it in practice”.

## Docs naming cheatsheet (quick map)
- DS: Datasheet (e.g., `001 DS8626 DataSheet stm32f407ve.pdf`) → electrical limits, clock sources.
- RM: Reference Manual (e.g., `002 rm0090-...pdf`) → registers/bitfields for all peripherals.
- PM: Programming Manual (e.g., `003 pm0214-...pdf`) → NVIC, exception model, MPU.
- ES: Errata Sheet (e.g., `005 es0182-...pdf`) → known issues + workarounds.
- AN: Application Note (e.g., `AN/an2834-...pdf`) → practical setup/accuracy/peripheral recipes.
- TN: Technical Note (`TN/*.pdf`) → packaging/thermal/production nuances.

## Answer pattern (keep it tight but deep + rich docs cites)
Genernal approach: 1) Big picture and goal → 2) Mechanism/dataflow → 3) Config checklist (registers/bits) → 4) Pitfalls & best practices → 5) Rich Doc cites (file + pages + topic/section).

- Scope & goal: State what the system must do (functional) and the non‑functional targets (rate, latency, resolution, ranges, power).
- Architecture map: Draw the dataflow and dependencies (core ↔ AHB/APB, TIM TRGO → ADC EXTSEL, ADC → DMA → buffer, buffer → control → TIMx_CCRy).
- Contract & constraints:
  - Inputs/Outputs: pins, channels, buses; buffer shape/width (8/16/32‑bit), length, ownership.
  - Timing math: clock tree (SYSCLK/AHB/APB), PLL factors, PSC/ARR → PWM freq, ADC sample time + conversion cycles → sample rate; USB must be 48 MHz.
  - Limits: APB1 ≤ 42 MHz, APB2 ≤ 84 MHz, ADC clock ≤ device max (F4: 36 MHz), DMA bandwidth/align.
  - Trade‑offs: latency vs. jitter vs. power (gate unused clocks).
- Config checklist (registers/bits): name exact regs and key fields.
  - RCC: enable AHBx/APBxENR for GPIO, TIMx, ADCx, DMAx; set PLL, prescalers.
  - GPIO: MODER/OSPEEDR/PUPDR/AFR[0/1] per signal (e.g., PA8→TIM1_CH1 AF1).
  - TIM: CR1, PSC/ARR, CCMR/CCER, BDTR (DTG/MOE), SMCR (TRGO/master/slave).
  - ADC: SMPRx, SQRx sequence, CR2 (DMA, CONT), EXTSEL/EXTEN for trigger.
  - DMA: SxCR (DIR, MSIZE/PSIZE, CIRC, DBM), NDTR, PAR, M0AR/M1AR, FCR.
  - NVIC: priority grouping, IRQ enables, preempt/subpriority.
- Bring‑up order: enable DMA → enable peripheral → arm trigger source (TIM/EXTI) to avoid spurious first events.
- Verification & observability: which flags/counters should advance and at what cadence.
  - TIMx_SR (UIF/CCxIF), ADCx_SR (EOC/EOS), DMA_LISR/HISR (HT/TC/TE), pointer movement.
  - Instrumentation: scope GPIO toggles; DWT->CYCCNT for latency; check overrun/underrun.
- Pitfalls & best practices: alignment (32‑bit→4‑byte), cache/consistency, short ISRs, deterministic priorities, ping‑pong for streaming, errata checks before conclusions.
- Doc cites: include file + page range + topic/section. Example pattern:
  - “RM0090, pp. XX–YY, RCC clocks (ENR/CFGR)”
  - “RM0090, pp. XX–YY, TIM advanced (BDTR/MOE/DTG)”
  - “RM0090, pp. XX–YY, ADC regular vs injected (EXTSEL/SMPR/SQR)”
  - “RM0090, pp. XX–YY, DMA (SxCR/NDTR/PAR/MxAR/FCR)”
  - “DS8626, pp. XX–YY, clock specs and limits”
  - “PM0214, pp. XX–YY, NVIC priority/faults”
  - “ES0182, pp. XX–YY, related errata/workaround”
  - “AN2834/AN4073, pp. XX–YY, ADC accuracy/noise tips”
  - “AN4013/AN4776, pp. XX–YY, timer triggers/PWM cookbook”
  - “AN4031, pp. XX–YY, DMA controller/FIFO”.
- Examples: keep generic guidance here; place concrete setups in “Worked examples” below.

Tiny response template:
- Big picture & goal → Architecture/dataflow → Contract (IO, timing math, limits) → Config checklist (regs/bits) → Bring‑up order → Verify (flags/scope) → Pitfalls → Doc cites.

## Why modules exist, how they work, what to configure
- Clock & RCC (why): Feed the core/peripherals with correct frequencies; gate power.
  - Configure: HSE/PLL, AHB/APB prescalers, 48 MHz for USB; enable per‑clk in RCC_AHBx/APBxENR.
  - Docs: RM0090 RCC; DS8626 clock specs; AN3988 (clock config tool).
  - Best practices: Derive USB at 48 MHz exactly; keep APB1 ≤ 42 MHz; gate unused clocks.

- Timers (why): PWM, input capture, motor control, timebase.
  - Configure: PSC/ARR, CHx mode (PWM/input), complementary outputs, break‑deadtime (advanced TIM1/TIM8).
  - Docs: RM0090 TIM; AN4013 (timers intro); AN4776 (timer cookbook).
  - Best practices: Compute PWM from clock tree; use one‑pulse or master/slave trigger chains for deterministic timing.

- ADC (why): Sense analog world; simultaneous sampling.
  - Configure: Channel sampling times, regular/injected sequences, DMA continuous/circular, triggers from TIM.
  - Docs: RM0090 ADC; AN2834 (optimize accuracy); AN4073 (F4 ADC accuracy tips).
  - Best practices: Use timer‑triggered DMA; stabilize Vref/Vdda; calibrate offset; route critical buffers away from cache.

- DMA (why): Offload CPU; sustain throughput.
  - Configure: Src/Dst, data width, circular, FIFO (where available), interrupt on half/complete.
  - Docs: AN4031 (DMA controller); RM0090 DMA.
  - Pitfalls: Alignment (32‑bit → 4‑byte aligned); don’t access buffers concurrently without barriers; prefer ping‑pong.

- Boot & system memory (why): Production programming and recovery.
  - Configure: BOOT0/BOOT1, option bytes; protocols via USART/SPI/I2C/USB DFU.
  - Docs: AN2606 (system memory boot); AN3155 (USART), AN3156 (USB DFU).

- NVIC & faults (why): Determinism and robustness.
  - Configure: Preempt/subpriority, group settings; fault handlers to capture stacked regs.
  - Docs: PM0214 (NVIC/faults); ARM TRM (lazy stacking, FPU context).
  - Best practices: Keep ISRs short; prioritize real‑time (PWM/safety) above comms; use WFI in idle.

- GPIO & power (why): IO correctness and energy.
  - Configure: MODER/OSPEEDR/PUPDR/AFR; clock gating in RCC AHB1ENR.
  - Docs: RM0090 GPIO; AN4899 (GPIO low‑power).

## Cross‑topic flows you should explain
- “ADC → DMA → buffer → control loop → PWM output”: Trigger from TIM, circular DMA, half‑buffer ISR, write compare to TIMx_CCRy.
- “USART bootloader update”: Enter system memory (AN2606), speak AN3155 protocol, verify via ES0182 constraints.
- “Low‑noise sampling”: Clock & GPIO layout (DS8626), ADC setup (AN2834/AN4073), DMA isolation, NVIC priorities.

## Debug & verification workflow
- When a peripheral “doesn’t work”, confirm: clock enabled → GPIO AF correct → trigger/routing set → status flags advance.
- Use AN4989 (Debug toolbox) for step‑by‑step bring‑up; always cross‑check ES0182 for errata before concluding behavior.
- For hard faults, dump stacked PC/LR/xPSR; see PM0214 for decoding.

## Bring‑up guideline (generic, reusable)
- Enable clocks: set RCC_AHBx/APBxENR; verify with peripheral SR/IDR reads; gate unused clocks.
- Configure GPIO: MODER/OTYPER/OSPEEDR/PUPDR and AFR[0/1] per signal role; confirm board pinout.
- Reset peripheral: pulse APBxRSTR bit; read back to ensure a clean baseline.
- Configure core mode: choose functional mode (e.g., TIM PWM/Input, ADC regular/injected), prescalers, sample times.
- Route triggers: select TRGO/EXTSEL (TIM→ADC, TIM master/slave), and ensure the DMA request line matches the peripheral.
- DMA setup: channel/stream, dir, width alignment (8/16/32‑bit), circular or ping‑pong, HT/TC interrupts; place buffers in SRAM2 if shared.
- NVIC: set group/priorities (real‑time > comms > background); keep ISRs short; use half‑buffer strategy for streaming.
- Start sequence: enable DMA → enable peripheral → arm trigger source (TIM/EXTI) in that order to avoid spurious events.
- Verify progress: check status flags & counters, observe HT/TC firing cadence, confirm pointers advance without overrun.
- Instrumentation: scope critical pins; optionally toggle a GPIO in ISR; use DWT->CYCCNT for latency; WFI in idle.

Exit criteria: data moves at expected rates without CPU polling; no overrun/underrun; ISR latency fits budget; power within target.

## Conventions for answers
- Use RM0090 register/bit names (e.g., RCC->CFGR.SW, TIM1->BDTR.MOE). Provide clock calculations when relevant.
- Cite docs by filename + page range + topic (e.g., “RM0090, pp. XX–YY, TIM advanced‑control (break‑deadtime)”; “AN4013, pp. XX–YY, timer triggers”).
- Prefer timer‑triggered DMA, gated clocks, and deterministic priorities; call out alignment, bus bandwidth, and power trade‑offs explicitly.

## Doc citation quick guide (add pages fast)
- Pattern: File, pp. A–B, Topic — mention key regs/bits/keywords you searched.
- Quick‑find keywords in PDFs (search these to locate pages):
  - RCC: “RCC_AHB1ENR”, “RCC_APB1ENR”, “RCC->CFGR”, “PLL”, “HSE”.
  - TIM advanced: “BDTR”, “MOE”, “DTG”, “OCxM”, “CCER”, “TRGO”, “SMCR”.
  - ADC: “EXTSEL”, “EXTEN”, “SMPR1/2”, “SQR1/2/3”, “CR2 DMA”, “injected”.
  - DMA: “SxCR”, “NDTR”, “PAR”, “M0AR/M1AR”, “FCR”, “stream/channel mapping”.
  - NVIC/faults: “NVIC IPR”, “priority grouping”, “AIRCR”, “HardFault”, “xPSR”.
  - Datasheet limits: “Maximum frequency”, “APB1 42 MHz”, “APB2 84 MHz”, “ADC clock 36 MHz”, “USB 48 MHz”.
  - Errata: search feature name + “limitation” (e.g., “ADC overrun”, “USB OTG FS”).
  - AN2834/AN4073: “layout”, “sampling time”, “impedance”, “oversampling/averaging”.
  - AN4013/AN4776: “master/slave mode”, “trigger output”, “PWM mode 1/2”.
  - AN4031: “FIFO threshold”, “burst”, “double buffer (DBM)”.

## Worked examples (rich, evidence‑driven for key points)

### General guidelines for examples
- Start with goal + timing/throughput target; list I/O, buffers, ownership.
- Show configuration with exact registers/bits and initial values.
- Prove mode/state by reading back registers/flags and interpreting them.
- Verify timing with math + observable counters/flags; instrument with scope/MCO/DWT.
- Document ISR/NVIC behavior (preemption, nesting) with ICSR/RETTOBASE evidence.
- Compare alternatives: what if you don’t use this feature? Note limitations/trade‑offs.
- Cite docs with file + pp. range + topic (see citation quick guide above).

### Example 1: Timer‑triggered ADC with circular DMA → control loop → PWM
- Configure
  - RCC: enable GPIOx/TIMx/ADCx/DMAx in RCC_AHB1ENR/APB2ENR; set PLL per clock math (USB=48 MHz).
  - TIMx: PSC/ARR for Fs, CR2.MMS=010 (TRGO on update) or per AN4013; optionally one‑pulse.
  - ADC: SMPRx per source impedance; SQR for sequence; CR2.DMA=1, DDS=1, CONT=0; EXTSEL=TIMx_TRGO, EXTEN=01 (rising).
  - DMA: SxCR.DIR=peripheral→memory, CIRC=1, MSIZE/PSIZE=16‑bit, MINC=1; NDTR=2N; PAR=&ADCx->DR; M0AR/M1AR to buffers.
- Evidence
  - Read TIMx_CR2.MMS, ADC_CR2.EXTSEL/EXTEN, DMA_SxCR.CIRC/MSIZE/PSIZE, RCC enable bits.
  - Observe DMA_LISR/HISR: HTIFx/TCIFx cadence; ADCx_SR: OVR must stay 0; TIMx_SR.UIF rate matches Fs.
  - Instrument: toggle GPIO in HT/TC ISR; or route TRGO cadence via another timer; measure with scope.
- Control loop
  - Run control on half‑buffer boundaries to meet latency/jitter constraints; update TIMy_CCRz for PWM.
- Alternatives & limits
  - Polling EOC: higher CPU load, jitter; may miss deadlines at high Fs.
  - ADC software trigger: non‑deterministic w.r.t. PWM; timer trigger removes phase drift.
- Docs: RM0090, pp. XX–YY (TIM CR2.MMS/TRGO; ADC EXTSEL/CR2.DMA; DMA SxCR/NDTR); AN4013 (triggers); AN2834/AN4073 (accuracy); AN4031 (DMA).

### Example 2: Complementary PWM with break‑deadtime (TIM1/TIM8)
- Configure
  - TIM1: PSC/ARR; CCMR1/2 OCxM=PWM1/2, OCxPE=1; CCER enable CHx + CHxN with proper polarity.
  - BDTR: DTG set per dead‑time; OSSR/OSSI as needed; BKP/BKF if used; MOE=1.
  - GPIO: map CHx (AF1) and CHxN to appropriate pins with speed=high, no pull.
- Evidence
  - Read TIM1_BDTR (MOE=1, DTG≠0), TIM1_CCER enable bits, TIM1_CCMR OCxM.
  - Scope: verify complementary outputs and measured dead‑time equals DTG programming.
  - Break test: pull BKIN active → TIM1_BDTR.BKE=1 should force outputs safe; read TIM1_SR for break flags.
- Alternatives & limits
  - Using general‑purpose timers + GPIO toggling cannot generate true complementary outputs with dead‑time; risk of shoot‑through.
- Docs: RM0090, pp. XX–YY (TIM advanced‑control/BDTR/MOE/DTG; CCER/CCMR); AN4776 (cookbook).

### Example 3: Nested interrupts and deterministic priorities
- Configure
  - PM0214: set priority grouping; assign preemption priorities so ISR_A preempts ISR_B.
  - NVIC: enable both IRQs; optional tail‑chaining demo.
- Evidence
  - Read SCB->ICSR.VECTACTIVE and RETTOBASE during ISRs to prove nesting; log timestamps with DWT->CYCCNT.
  - Check NVIC->IPR values match intended preemption/subpriority; confirm group in SCB->AIRCR.PRIGROUP.
  - Show stacked LR/PC in the preempted ISR prologue.
- Alternatives & limits
  - Single priority level → no preemption → increased latency for critical control loops.
  - Excessive nesting → stack pressure; measure maximum depth vs. stack size.
- Docs: PM0214, pp. XX–YY (NVIC priority grouping, ICSR, RETTOBASE, tail‑chaining); ARM TRM (lazy stacking/FPU context).

### Example 4: DMA double‑buffer (DBM) vs circular single‑buffer
- Configure
  - Enable DBM in DMA_SxCR; set M0AR and M1AR; use HT/TC to alternate halves; watch SxCR.CT toggle.
- Evidence
  - Read DMA_SxCR.DBM=1; observe SxCR.CT toggling; NDTR reload at boundaries; no TE/FE/ DME errors in LISR/HISR.
- Alternatives & limits
  - Single circular buffer: simpler but higher risk of producer/consumer contention; requires careful HT/TC handling.
  - FIFO off vs on (FCR): affects burst efficiency; misconfig can throttle throughput.
- Docs: RM0090/AN4031, pp. XX–YY (DBM, FIFO, HT/TC semantics).

### Example 5: Encoder interface using timer in encoder mode
- Configure
  - TIMx_SMCR.SMS=001/010 (encoder mode), TI1/TI2 mapped as inputs; CCMR configured as inputs; IC filters per noise.
- Evidence
  - Move shaft: TIMx_CNT increments/decrements; read TIMx_SR for over/underflow; verify direction via TIMx_CR1.DIR.
- Alternatives & limits
  - Polling EXTI edges: higher CPU load, worse jitter; lost edges at speed; no built‑in direction decode.
- Docs: RM0090, pp. XX–YY (TIM encoder mode, SMCR/CCMR/CCER).

### Example 6: Low‑noise ADC sampling (layout + timing)
- Configure
  - Increase sampling time (SMPRx) for high source impedance; average/oversample in software; trigger via TIM to avoid jitter.
  - Power: stabilize VDDA/Vref; reduce digital noise (gate clocks not used).
- Evidence
  - Measure code dispersion (stddev) before/after changes; confirm ADC clock ≤ 36 MHz; verify trigger determinism.
- Alternatives & limits
  - Short sampling with high‑Z source → gain error/noise; software averaging adds latency.
- Docs: AN2834/AN4073, pp. XX–YY (accuracy, impedance, oversampling); DS8626 limits.

### Example 7: Clock math and validation (SYSCLK/USB/APB)
- Configure
  - Set PLLM/N/P/Q to achieve SYSCLK target and USB=48 MHz; configure Flash latency per SYSCLK.
- Evidence
  - Read RCC->CFGR.SWS=PLL; RCC->PLLCFGR values; verify APB1≤42 MHz, APB2≤84 MHz.
  - Output MCO on a pin (RCC->CFGR.MCOx) and measure frequency with scope.
- Alternatives & limits
  - Wrong Flash wait‑states → hard faults/data corruption; wrong USB clock → USB enumeration fails.
- Docs: DS8626/RM0090, pp. XX–YY (clock tree, PLL, Flash latency); AN3988 (tooling).

### Example 8: Fault triage with stacked registers (HardFault/BusFault)
- Configure
  - Install HardFault/BusFault handlers that capture stacked r0‑r3, r12, lr, pc, xPSR; read CFSR/HFSR/BFAR/MMFAR.
- Evidence
  - Report PC/LR to locate faulting instruction; decode CFSR to identify precise cause (e.g., PRECISERR, UNALIGNED);
  - If lazily stacked FPU: consult PM0214 notes.
- Alternatives & limits
  - Without structured dump, fault root cause opaque; random resets during development.
- Docs: PM0214/ARM TRM, pp. XX–YY (fault model, stacked frame).

### Example 9: System bootloader over USART (field update)
- Configure
  - Enter system memory per AN2606 (BOOT0 pin/OB settings); use AN3155 command set (INIT/GET/ERASE/WRITE/VERIFY).
- Evidence
  - Observe bootloader responses (ACK/bytes) per AN3155; confirm flash contents after programming.
- Alternatives & limits
  - Custom app‑level protocol requires your own ROM/DFU; more maintenance; check ES0182 limitations.
- Docs: AN2606/AN3155/AN3156/ES0182, pp. XX–YY.