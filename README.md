# Multi-Vessel UAV Routing with Time-Dependent Profit

This repository provides the ALNS heuristic implementation for the multi-vessel UAV routing problem with time-dependent profit. Exact optimization code, including the LBBD implementation, is maintained separately and is not part of this public code release.

## Repository layout

- `ALNS/ALNS/`: C++ ALNS source and Visual Studio project files.
- `ALNS/ALNS.sln`: Visual Studio solution.
- `ALNS/README_ALNS_YISHEN.md`: ALNS implementation notes and experiment configuration.

Instance data and generated experiment outputs are intentionally excluded. Obtain the instance files from the public data repository associated with the study, then place them under `ALNS/Data(4.4)/` using the names `Example<num1>_<num2>_<num3>.dat`. The ALNS executable expects the `ALNS` directory as its working directory and writes results under `ALNS/results/`.

## Build

Open `ALNS/ALNS.sln` in Visual Studio and build the `x64` `Release` configuration. Configure the CPLEX include and library paths for your local installation.

## Reproducibility

See `ALNS/README_ALNS_YISHEN.md` for available environment variables, operator ablations, and recommended experiment reporting. Solver versions and instance data are managed separately from this code repository.
