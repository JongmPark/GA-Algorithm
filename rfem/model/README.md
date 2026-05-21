# MuJoCo Model Files

This directory contains the physics model definitions, parameter configurations, and motion-related data used in the Genetic Algorithm optimization framework.

The models are designed to reproduce RFEM-based rod/pendulum dynamics inside the MuJoCo simulation environment.

---

# Folder Contents

```text
models/
│
├── pendulum_5.xml
├── pendulum_5.urdf
├── flexibility_params.yml
├── ga_algorithm.py
├── motion_data.csv
├── simulation_log.csv
└── README.md

# File Descriptions

## `pendulum_5.xml`

Main MuJoCo simulation model file.

### Contains:

- Multi-body pendulum structure
- Joint definitions
- Damping and stiffness parameters
- Inertial properties
- Visualization settings
- Camera and lighting configuration

The XML model defines a chained rod system composed of multiple connected rigid bodies.

### Key Parameters:

- `stiffness`
- `damping`
- `mass`
- `joint range`
- `inertia`

### Example Structure:

- 6 connected rod segments
- Revolute joints
- Physics-based dynamic simulation

---

## `pendulum_5.urdf`

URDF representation of the pendulum model.

### Used for:

- Robot structure definition
- Cross-platform compatibility
- Simulation interoperability
- Model visualization

### This file describes:

- Links
- Joints
- Collision geometry
- Kinematic hierarchy

---

## `flexibility_params.yml`

Configuration file containing flexible body parameters.

### Stores:

- Damping coefficients
- Stiffness coefficients
- Material-related parameters
- Simulation tuning values

This file allows parameter modification without directly editing the simulation model.

---

## `ga_algorithm.py`

Genetic Algorithm optimization script.

### Responsibilities:

- Population initialization
- Fitness evaluation
- Selection
- Crossover
- Mutation
- Parameter evolution

The algorithm searches for optimal D/K parameter combinations that minimize the difference between simulated motion and RFEM reference motion.

---

## `motion_data.csv`

Recorded motion trajectory data.

### Typically includes:

- Time step
- Joint position
- Velocity
- Angular displacement
- End-effector trajectory

### Used for:

- Fitness calculation
- Motion comparison
- Visualization
- Error analysis

---

## `simulation_log.csv`

Simulation execution records.

### Contains:

- Generation number
- Fitness score
- Selected parameters
- Optimization history
- Convergence information

### Useful for:

- Debugging
- Optimization tracking
- Performance analysis

---

# Simulation Overview

```text
RFEM Reference Motion
        │
        ▼
 MuJoCo Simulation
        │
        ▼
 Motion Data Generation
        │
        ▼
 Fitness Evaluation
        │
        ▼
 Genetic Algorithm Optimization
```

---

# Main Parameters

| Parameter | Description |
|---|---|
| D | Damping coefficient |
| K | Stiffness coefficient |
| Fitness | Motion similarity error |
| Generation | GA iteration count |

---

# Research Purpose

This folder supports research on:

- Digital Twin systems
- Physics simulation optimization
- Parameter identification
- Evolutionary optimization
- Motion similarity analysis

---

# Notes

- The MuJoCo XML model is the primary simulation environment.
- RFEM motion data is treated as ground-truth reference data.
- Genetic Algorithm optimization attempts to minimize trajectory error.
- Parameter tuning focuses mainly on damping and stiffness estimation.
