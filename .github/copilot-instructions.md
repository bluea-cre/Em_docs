# Embedded Systems Documentation Repository

This is a comprehensive documentation repository for ARM Cortex-M4 and STM32F407VET6 embedded systems development.

## Repository Structure

The documentation is organized hierarchically by processor architecture and specific microcontroller:

- **`ARM/`** - Core ARM Cortex-M4 processor documentation
  - Processor technical reference manuals (TRM)
  - Architecture reference manuals (ARMv7-M)
  - Processor datasheets and comparison tables
  - Software developer errata notices
  - Lazy stacking and context switching application notes

- **`STM32F407VET6/`** - STMicroelectronics STM32F407VET6 specific documentation
  - **Root level**: Core datasheets, reference manuals, programming manuals, and board images
  - **`AN/`**: Application notes covering peripherals, protocols, and implementation guides
  - **`AN-Tool&Software/`**: Tool-specific and software development application notes
  - **`Others/`**: Marketing materials, presentations, certifications, and security bulletins
  - **`TN/`**: Technical notes covering packaging, shipping, and device marking

## Document Categories and Naming Convention

Documents follow STMicroelectronics naming conventions:
- **DS** prefix: Datasheets (e.g., `DS8626 DataSheet stm32f407ve.pdf`)
- **RM** prefix: Reference manuals (e.g., `rm0090-stm32f405415...pdf`)
- **PM** prefix: Programming manuals (e.g., `pm0214-stm32-cortexm4-mcús...pdf`)
- **AN** prefix: Application notes with sequential numbering (e.g., `an2606-introduction-to-system-memory...pdf`)
- **TN** prefix: Technical notes (e.g., `tn1163-description-of-wlcsp...pdf`)
- **ES** prefix: Errata sheets (e.g., `es0182-stm32f405407xx...pdf`)

## Key Documentation Priorities

When working with this repository, prioritize these critical documents:

1. **Core Architecture**: Start with ARM processor TRM and architecture reference manual
2. **MCU Specifics**: STM32F407VET6 datasheet and reference manual for register-level details
3. **Programming**: Cortex-M4 programming manual for instruction set and system control
4. **Application Notes**: Focus on peripheral-specific AN documents for implementation guidance

## Development Context Patterns

- **Boot Modes**: Reference AN2606 for system memory boot mode understanding
- **Peripheral Configuration**: Use AN documents for specific peripheral implementations (ADC, USART, SPI, etc.)
- **Power Management**: Consult power-related AN documents for low-power design patterns
- **Communication Protocols**: Leverage protocol-specific AN documents (USB, CAN, I2C, SPI)
- **Development Tools**: Reference AN-Tool&Software for STM32CubeIDE, STM32CubeMX integration patterns

## Common Technical Workflows

- **Register Programming**: Always cross-reference datasheet register maps with reference manual bit descriptions
- **Peripheral Initialization**: Follow AN document code examples for proper peripheral setup sequences
- **Debugging**: Use processor errata notices to identify known silicon limitations
- **Hardware Design**: Consult packaging and thermal management technical notes for PCB design

## File Access Patterns

- PDFs contain detailed technical specifications requiring document viewer integration
- Board images (`.jpg`, `.avif`) provide visual pin layout reference
- Sequential AN numbering indicates chronological development and superseding documents
- Higher AN numbers typically contain more recent recommendations and updated methodologies

When suggesting code implementations or hardware configurations, always reference the specific document section that validates the approach.