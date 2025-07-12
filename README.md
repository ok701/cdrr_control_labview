# Adaptive Impedance Control for Upper-Limb Cable-Driven Rehabilitation Robot

For full technical details and experimental results, please refer to the complete thesis [here](./GIST_Dissertation.pdf).


## Overview

This project presents an **Adaptive Impedance Control Strategy** for an upper-limb **Cable-Driven Rehabilitation Robot (CDRR)**. The controller combines assistive and resistive behaviors using a novel *Assist-Resist Adaptive Control (ARAC)* algorithm, dynamically adjusting stiffness and damping in real-time based on user performance. The system aims to deliver personalized, phase-specific rehabilitation for bedridden patients with upper limb impairments.


## Key Features

- Assist-Resist Adaptive Control (ARAC) combining Assist-as-Needed (AAN) and Resist-as-Needed (RAN)
- Adaptive learning using Radial Basis Function Networks (RBFN) for real-time stiffness and damping tuning
- Phase-specific impedance modulation within a single movement cycle
- Custom-built 3-DOF planar CDRR with real-time force sensing
- Real-time control architecture using NI CompactRIO and LabVIEW GUI

## System Architecture

- **Motors**: 3 Maxon EC-I 52 DC motors with planetary gearheads  
- **Sensors**: Inline load cells, encoders, and series springs  
- **Control System**: Real-time control with NI CompactRIO  
- **Control Loops**:
  - Low-Level: Tension control at 500 Hz  
  - Mid-Level: Kinematics and force distribution at 100 Hz  
  - High-Level: Adaptive impedance learning at 50 Hz  


## Experimental Validation

- **Participants**: 8 individuals with simulated motor impairment using elastic bands  
- **Outcomes**:
  - ARAC increased muscular activation compared to AAN and Passive modes  
  - Maintained full range of motion across all participants  
  - Dynamically applied resistance only when users were capable  
