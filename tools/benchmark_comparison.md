# MoveIt Collision & Planning Benchmark Comparison

Comparing MoveIt performance **Before Optimization** (original high-density collision meshes) vs **Optimized** (decimated collision meshes).

## 1. Direct Collision Detection Performance (FCL Microbenchmark)
| Metric | Before Optimization | Optimized | Speedup / Improvement |
| :--- | :--- | :--- | :--- |
| **Collision Checks / Sec** | 18,711 Hz | 40,335 Hz | **2.16x faster** |
| **Mean Check Time** | 53.03 μs (0.0530 ms) | 24.49 μs (0.0245 ms) | **2.17x (+53.8%)** |
| **Median Check Time** | 25.47 μs | 22.96 μs | **1.11x (+9.8%)** |
| **95th Percentile Time** | 140.67 μs | 36.49 μs | **3.86x (+74.1%)** |
| **99th Percentile Time** | 745.37 μs | 49.04 μs | **15.20x (+93.4%)** |
| **Total Time (5,000 checks)** | 534.44 ms | 247.93 ms | **2.16x (+53.6%)** |
| **Collision Detection Consistency** | 2786 / 10000 (27.9%) | 2861 / 10000 (28.6%) | Consistent |

### Detailed Contact Computation (1,000 queries with contact points & normals):
| Metric | Before Optimization | Optimized | Speedup |
| :--- | :--- | :--- | :--- |
| **Mean Contact Time** | 96.67 μs | 34.27 μs | **2.82x (+64.6%)** |
| **Median Contact Time** | 29.20 μs | 26.93 μs | **1.08x (+7.8%)** |
| **95th Percentile** | 493.63 μs | 69.81 μs | **7.07x (+85.9%)** |

## 2. MoveIt Motion Planning Time Comparison (OMPL / RRTConnect)

### Planning Group: `both_arms`
| Scenario | Before Mean (ms) | Before Median (ms) | Optimized Mean (ms) | Optimized Median (ms) | Speedup (Mean) | Success Rate |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Home -> Standby** | 28.70 ms | 30.33 ms | **26.65 ms** | **27.55 ms** | **1.08x (+7.2%)** | 50/50 vs 50/50 |
| **Standby -> Basket** | 42.18 ms | 38.38 ms | **23.71 ms** | **22.78 ms** | **1.78x (+43.8%)** | 50/50 vs 50/50 |
| **Standby -> Pre-place** | 21.12 ms | 19.82 ms | **19.80 ms** | **19.01 ms** | **1.07x (+6.3%)** | 50/50 vs 50/50 |
| **Pre-place -> Basket** | 44.20 ms | 40.75 ms | **24.98 ms** | **22.77 ms** | **1.77x (+43.5%)** | 50/50 vs 50/50 |
| **Basket -> Home** | 34.36 ms | 27.06 ms | **20.96 ms** | **19.64 ms** | **1.64x (+39.0%)** | 50/50 vs 50/50 |
| **Home -> Hands-up** | 21.13 ms | 18.70 ms | **20.78 ms** | **19.24 ms** | **1.02x (+1.7%)** | 50/50 vs 50/50 |
| **Group Average** | **31.95 ms** | - | **22.81 ms** | - | **1.40x (+28.6%)** | - |

### Planning Group: `left_arm`
| Scenario | Before Mean (ms) | Before Median (ms) | Optimized Mean (ms) | Optimized Median (ms) | Speedup (Mean) | Success Rate |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Home -> Standby** | 23.18 ms | 20.27 ms | **21.65 ms** | **18.82 ms** | **1.07x (+6.6%)** | 50/50 vs 50/50 |
| **Standby -> Basket** | 51.00 ms | 46.83 ms | **21.39 ms** | **20.23 ms** | **2.38x (+58.1%)** | 50/50 vs 50/50 |
| **Standby -> Pre-place** | 21.44 ms | 18.75 ms | **19.83 ms** | **18.58 ms** | **1.08x (+7.5%)** | 50/50 vs 50/50 |
| **Pre-place -> Basket** | 52.42 ms | 50.22 ms | **23.51 ms** | **21.51 ms** | **2.23x (+55.1%)** | 50/50 vs 50/50 |
| **Basket -> Home** | 26.99 ms | 24.20 ms | **20.42 ms** | **17.63 ms** | **1.32x (+24.3%)** | 50/50 vs 50/50 |
| **Home -> Hands-up** | 18.47 ms | 16.87 ms | **19.64 ms** | **17.59 ms** | **0.94x (-6.3%)** | 50/50 vs 50/50 |
| **Group Average** | **32.25 ms** | - | **21.07 ms** | - | **1.53x (+34.7%)** | - |

### Planning Group: `right_arm`
| Scenario | Before Mean (ms) | Before Median (ms) | Optimized Mean (ms) | Optimized Median (ms) | Speedup (Mean) | Success Rate |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Home -> Standby** | 23.25 ms | 20.43 ms | **22.31 ms** | **19.74 ms** | **1.04x (+4.1%)** | 50/50 vs 50/50 |
| **Standby -> Basket** | 19.11 ms | 18.39 ms | **19.87 ms** | **19.01 ms** | **0.96x (-4.0%)** | 50/50 vs 50/50 |
| **Standby -> Pre-place** | 21.48 ms | 24.41 ms | **19.49 ms** | **16.69 ms** | **1.10x (+9.3%)** | 50/50 vs 50/50 |
| **Pre-place -> Basket** | 19.38 ms | 18.97 ms | **19.87 ms** | **19.14 ms** | **0.98x (-2.5%)** | 50/50 vs 50/50 |
| **Basket -> Home** | 18.83 ms | 16.06 ms | **19.98 ms** | **17.71 ms** | **0.94x (-6.1%)** | 50/50 vs 50/50 |
| **Home -> Hands-up** | 19.82 ms | 16.99 ms | **21.74 ms** | **24.44 ms** | **0.91x (-9.7%)** | 50/50 vs 50/50 |
| **Group Average** | **20.31 ms** | - | **20.54 ms** | - | **0.99x (-1.1%)** | - |

## 3. Overall Benchmark Summary
| Metric | Before Optimization | Optimized | Improvement |
| :--- | :--- | :--- | :--- |
| **Average Planning Time Across All Scenarios** | **28.17 ms** | **21.48 ms** | **1.31x (+23.8%)** |
| **Collision Checking Speed** | **53.03 μs** | **24.49 μs** | **2.17x (+53.8%)** |
| **Collision Checking Frequency** | **18,711 checks/s** | **40,335 checks/s** | **2.16x higher** |