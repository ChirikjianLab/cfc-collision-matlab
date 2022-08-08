# CFC: Collision detection based on closed-form contact space parameterization (MATLAB version)
Collision detection, distance queries (penetration depth), closes points computations via closed-form contact space (CFC) for unions of convex bodies with smooth boundaries. MATLAB implementation, including test scripts and visualization of the results from C++ implementation.

## Introduction
This is the MATLAB implementation for the narrow phase collision detection problem between two general unions of convex bodies encapsulated by smooth surfaces. The approach, namely CFC (Closed-Form Contact space), is based on parameterizing their contact space in closed-form. The first body is dilated to form the contact space while the second is shrunk to a point. Then, the collision detection is formulated as finding the closest point on the parametric contact space with the center of the second body. Numerical solutions are proposed based on the point-to-surface distance as well as the common-normal concept. Furthermore, when the two bodies are moving or under linear deformations, their first time of contact is solved continuously along the time-parameterized trajectories. Benchmark studies are conducted for the proposed algorithms in terms of solution stability and computational cost.

- Paper: [IEEE Robotics and Automation Letters (RA-L)](https://ieeexplore.ieee.org/document/9829274)
- Project page: [https://chirikjianlab.github.io/cfc-collision/](https://chirikjianlab.github.io/cfc-collision/)
- C++ implementation: [https://github.com/ChirikjianLab/cfc-collision.git](https://github.com/ChirikjianLab/cfc-collision.git)
- Data: [Benchmark data in the paper](https://drive.google.com/drive/folders/17jSSC-EIhiSTqXSgfoEOs4R7mzKy1d1i?usp=sharing)

## Authors
[Sipu Ruan](https://ruansp.github.io), Xiaoli Wang and [Gregory S. Chirikjian](https://scholar.google.com/citations?user=qoIuyMoAAAAJ&hl=en)

## Clone the repository and submodule
```
git clone https://github.com/ruansp/cfc_collision_matlab.git
cd cfc_collision_matlab/
git submodule update --init
```

## Running tests and demonstrations
All the scripts are located in `/test` folder.

### Demonstration scripts
- [`demo_geometry.m`](/test/demo_geometry.m): Demonstration of the geometric bodies and closed-form contact space (CFC). Reproduction of __Fig. 1__ in the paper.
- [`demo_collision_detection.m`](/test/demo_collision_detection.m): Demonstration of the static collision detection using the proposed CFC-based algorithm. Reproduction of __Fig. 2__ in the paper.
- [`demo_collision_detection_continuous.m`](/test/demo_collision_detection_continuous.m): Demonstration of the continuous collision detection using the proposed CFC-based algorithm. Reproduction of __Fig. 3__ and __Fig. 4__ in the paper. Set parameter `separated = true` for __Fig. 3(a)__ and __Fig. 4(a)__; set `separated = false` for __Fig. 3(b)__ and __Fig. 4(b)__.
- [`demo_benchmark_data.m`](/test/demo_benchmark_data.m): Demonstration of the benchmark results from C++ implementation for running time, accuracy and number of iterations among different solvers. Reproduction of __Fig. 5__ in the paper. Please refer to the [README](/data/README.md) file in [`/data`](/data) folder for running instructions.

### Test scripts for implementations
- [`run_all_tests.m`](/test/run_all_test.m): Top-level script to run all the tests.
- [`test_collision_cfc.m`](/test/test_collision_cfc.m): Script for collision detection using the proposed CFC-based methods. Algorithms include _CFC-CN-FP_ and _CFC-Dist-LS_.
- [`test_collision_common_normal_fixed_point.m`](/test/test_collision_common_normal_fixed_point.m): Script for collision detection using common-normal concept, solved by fixed-point iteration method (_CN-FP_ algorithm).
- [`test_collision_implicit.m`](/test/test_collision_implicit.m): Script for collision detection using implicit surface expression, solved by interior-point method (_Implicit_ algorithm).
- [`test_continuous_collision_SQ.m`](/test/test_continuous_collision_SQ.m): Script for continuous collision detection using the proposed _CFC-Dist-LS_ algorithm. Comparison with the naive method by solving static collision detection at discretized time steps are conducted and visualized. User can choose the animation mode for viewing the whole movements.
- [`test_cost_3D.m`](/test/test_cost_3D.m): Script for visualizing cost function values of the static case in the parameter space for _CFC-Dist-LS_ algorithm.
- [`test_cost_ccd_3D.m`](/test/test_cost_ccd_3D.m): Script for visualizing cost function values of the continuous case in the time space for _CFC-Dist-LS_ algorithm.
- [`test_geometry_SQ.m`](/test/test_geometry_SQ.m): Script for visualizing superquadric geometric model, including its body shape, outward-pointing normal and contact space with another superquadric.

### Benchmark scripts
- [`benchmark_SQ_3D.m`](/test/benchmark_SQ_3D.m): Benchmark among different algorithms for the SQ-SQ collision detection.
- [`main_SQ_3D.m`](/test/main_SQ_3D.m): Benchmark among different algorithms for the SQ-SQ collision detection. Visualization of the bodies for each trial is provided.

## Reference
If you find our work useful in your research, please consider citing:

- S. Ruan, X. Wang and G. S. Chirikjian, "Collision Detection for Unions of Convex Bodies With Smooth Boundaries Using Closed-Form Contact Space Parameterization," in IEEE Robotics and Automation Letters, vol. 7, no. 4, pp. 9485-9492, Oct. 2022, doi: 10.1109/LRA.2022.3190629.

- BibTeX
```
@ARTICLE{9829274,
  author={Ruan, Sipu and Wang, Xiaoli and Chirikjian, Gregory S.},
  journal={IEEE Robotics and Automation Letters}, 
  title={Collision Detection for Unions of Convex Bodies With Smooth Boundaries Using Closed-Form Contact Space Parameterization}, 
  year={2022},
  volume={7},
  number={4},
  pages={9485-9492},
  doi={10.1109/LRA.2022.3190629}}
```
