# CNS Flight Stack: Estimator Supervisor

[![License](https://img.shields.io/badge/License-AAUCNS-336B81.svg)](./LICENSE) [![Paper](https://img.shields.io/badge/IEEEXplore-10.1109/LRA.2022.3196117-00629B.svg?logo=ieee)](https://doi.org/10.1109/LRA.2022.3196117) [![Release](https://img.shields.io/github/v/release/aau-cns/estimator_supervisor?include_prereleases&logo=github)](https://github.com/aau-cns/estimator_supervisor/releases)

Basic ROS node to check the deviation of an estimate over time and return false if it is larger than a predefined theshold.

Maintainer: [Alessandro Fornasier](mailto:alessandro.fornasier@aau.at)

## Credit
This code was written by the [Control of Networked System (CNS)](https://www.aau.at/en/smart-systems-technologies/control-of-networked-systems/), University of Klagenfurt, Klagenfurt, Austria.

## License
This software is made available to the public to use (_source-available_), licensed under the terms of the BSD-2-Clause-License with no commercial use allowed, the full terms of which are made available in the `LICENSE` file. No license in patents is granted.

### Usage for academic purposes
If you use this software in an academic research setting, please cite the
corresponding paper and consult the `LICENSE` file for a detailed explanation.

```latex
@article{cns_flightstack22,
    title        = {CNS Flight Stack for Reproducible, Customizable, and Fully Autonomous Applications},
    author       = {Scheiber, Martin and Fornasier, Alessandro and Jung, Roland and Böhm, Christoph and Dhakate, Rohit and Stewart, Christian and Steinbrener, Jan and Weiss, Stephan and Brommer, Christian},
    journal      = {IEEE Robotics and Automation Letters},
    volume       = {7},
    number       = {4},
    year         = {2022},
    doi          = {10.1109/LRA.2022.3196117},
    url          = {https://ieeexplore.ieee.org/document/9849131},
    pages        = {11283--11290}
}
```

## Getting Started

### Prerequesites
This package is part of the [CNS Flight Stack] and thus depends on the other packages of the flight stack:
- [CNS Flight Stack: Autonomy Engine]

Further the following libraries are required
- Eigen
- ROS noetic

### Build

As this is a ROS package, please build it within the catkin environment with

```bash
catkin build estimator_supervisor
```

## Usage
The intended usage is together with the [CNS Flight Stack: Autonomy Engine], which will interact with the supervisor. Use the provided launchfile to start the ROS1 node

```bash
roslaunch estimator_supervisor supervisor.launch
```

### ROS Parameters
This node uses the following parameters that can be set in any launch file.

| ROS parameter | description | required |
|:-------------:|:-----------:|:-------------:|
| `Supervisor_window_s` | time to supervise estimates in s | yes |
| `max_norm_changes`    | maximum norm change allowed for successful pass in position | yes |
| `topic_to_supervise`  | ROS topic used for supervision | yes |
| `estiamte_msg_type`   | ROS type of message used (available: `posestamped` or `posewithcovariancestamped`) | yes |

### Default Launchfile Parameters

These are the same as the ROS parameters, so only their default value is given.

| Launch parameter | default value                  |
|:----------------:|:------------------------------:|
| `Supervisor_window_s` | `10.0` |
| `max_norm_changes`    | `1.0` |
| `topic_to_supervise`  | `/ov_msckf/poseimu` |
| `estiamte_msg_type`   | `posestamped` |

## Architecture

Please refer to the academic paper for further insights of the Data Recorder.

## Package Layout

```console
/path/to/data_recorder$ tree -L 3 --noreport --charset unicode
.
|-- CMakeLists.txt
|-- launch
|   `-- recorder.launch
|-- LICENSE
|-- nodes
|   `-- DataRecorderNode.py
|-- package.xml
|-- README.md
|-- scripts
|   |-- record.sh
|   |-- stop_record.sh
|   `-- store.sh
|-- setup.py
`-- src
    `-- data_recorder
        |-- data_recorder.py
        `-- __init__.py
```

---

Copyright (C) 2021-2023 Alessandro Fornasier and Martin Scheiber, Control of Networked Systems, University of Klagenfurt, Austria.
You can contact the authors at [alessandro.fornasier@aau.at](mailto:alessandro.fornasier@aau.at?subject=[CNS%20Flight%20Stack]%20estimator_supervisor%20package), [martin.scheiber@aau.at](mailto:martin.scheiber@aau.at?subject=[CNS%20Flight%20Stack]%20estimator_supervisor%20package).

<!-- LINKS: -->
[CNS Flight Stack]: https://github.com/aau-cns/flight_stack
[CNS Flight Stack Scripts]: https://github.com/aau-cns/flight_stack/tree/main/src/flightstack/flightstack_scripts/record_scripts
[CNS Flight Stack: Autonomy Engine]: https://github.com/aau-cns/autonomy_engine
