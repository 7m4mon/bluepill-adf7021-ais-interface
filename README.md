# AIS Plotter Test Interface

A small bench-test interface for feeding simulated AIS targets into an AIS receiver or chart plotter.

<img width="1926" height="2205" alt="Image" src="https://github.com/user-attachments/assets/ad5c90ec-f1c4-4f88-93ef-8c4624801fd4" />

## Why I Built It

I originally looked at using a commercial AIS test unit such as the Quark-elec AT011.

However, its USB virtual COM port is intended for the vendor's configuration software, and it does not provide a public command/API for injecting arbitrary vessel data from an external scenario generator.

Because the goal was to control simulated AIS targets from a PC in real time, I decided to build the RF interface myself.

## Purpose

The goal of this project is **not to build a general-purpose AIS transmitter**.

It is intended as the RF output stage of an AIS plotter test environment, where target position, speed, course, CPA/TCPA scenarios, and collision-warning cases are generated on a PC and then presented to real AIS equipment.

## Hardware

- STM32F103CB "Bluepill"
- RF7021SE module
- Analog Devices ADF7021
- 19.68 MHz reference
- External VCO in the AIS band
- 128x32 SSD1306 OLED
- USB serial input
- Separate debug UART

## Data Flow

```text
PC / Scenario Generator
        |
        | USB serial
        v
STM32F103
        |
        | AIS framing
        | FCS / bit stuffing / NRZI
        v
ADF7021 / RF7021SE
        |
        v
Closed RF test setup
        |
        v
AIS receiver / chart plotter
```

## Notes

The prototype was verified with an RTL-SDR and AIS-catcher in a closed test setup.

The ADF7021 Gaussian filter uses BT=0.5, while AIS specifies BT=0.4, so this project should be treated as a **bench-test signal source**, not a standards-compliant AIS transmitter.

## Publication Policy

This repository documents the hardware architecture, signal-processing flow, and plotter-test concept.

It does **not** provide a ready-to-use firmware image or a turn-key procedure for transmitting arbitrary vessel identities and positions over public AIS channels.

The reusable part of the project is the **AIS target / scenario generation and plotter evaluation workflow**.

## Safety

AIS frequencies are used for maritime safety.

Any RF testing must be performed only in a closed test environment using suitable shielding, attenuation, dummy loads, or direct coupling, and in accordance with local regulations.
