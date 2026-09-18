# RAL-MultiRRT Reproducible Experiment Report

## Scope
This package rebuilds the experiments as a clean-room, fully executable 2D geometric planning benchmark. The original uploaded manuscript did not contain the planner implementation or complete fixed-map goal coordinates, so the maps in this package are newly constructed and explicitly stored. Results below are generated from the included code; no legacy numerical results are copied.

## Main experimental design
- Five algorithms: MultiRRT, RRT*, RRT*-Smart, APF-MultiRRT, RAL-MultiRRT.
- Four fixed maps, 20 independent planner seeds per algorithm/map (400 path-set attempts).
- Four random-map groups, 15 independently generated maps per group and algorithm (300 path-set attempts).
- Every path set contains three independent start-to-goal paths.
- A shared collision/boundary validator and safety radius are used for all algorithms.
- Turning statistics are computed after common 15 m arc-length resampling.
- Raw per-attempt counters include collision checks, validator calls, APF calls, look-ahead trials, samples, and expanded nodes.

## Fixed-map summary
```
map_group    algorithm  attempts  valid  success_rate  median_length  iqr_length  median_turning  median_clearance  median_runtime  median_collision_checks
       S1  APFMultiRRT      20.0   20.0          1.00    1080.410182   34.604581        0.033589          1.289767        0.158187                   2510.0
       S1     MultiRRT      20.0   11.0          0.55    1096.726559   82.976666        0.040015          1.689384        0.031017                    871.5
       S1  RALMultiRRT      20.0   20.0          1.00    1075.236436   19.308385        0.036136          1.871686        0.643479                   6407.0
       S1      RRTStar      20.0   10.0          0.50    1089.809797   58.323550        0.032360          1.388700        0.038114                    998.5
       S1 RRTStarSmart      20.0   11.0          0.55    1099.841817   74.366812        0.032322          1.733708        0.045472                   1215.0
       S2  APFMultiRRT      20.0   19.0          0.95    1054.865606   32.055231        0.033341          1.829421        0.139873                   2256.5
       S2     MultiRRT      20.0   13.0          0.65    1106.034587   71.564866        0.036627          1.310873        0.037601                    840.0
       S2  RALMultiRRT      20.0   20.0          1.00    1040.123319   22.729213        0.029677          1.923878        0.744847                   6651.0
       S2      RRTStar      20.0   13.0          0.65    1109.264497   94.638730        0.031615          1.310765        0.049708                    977.5
       S2 RRTStarSmart      20.0   13.0          0.65    1109.264497   98.958597        0.029224          1.310765        0.052092                   1187.5
       S3  APFMultiRRT      20.0   15.0          0.75     772.674545    9.701191        0.018685          2.066668        0.363261                   6343.0
       S3     MultiRRT      20.0    5.0          0.25     785.782059   22.227641        0.022240          0.579890        0.017486                    402.0
       S3  RALMultiRRT      20.0   18.0          0.90     793.811982   24.740720        0.028231          1.687936        0.950311                   8926.5
       S3      RRTStar      20.0    5.0          0.25     778.637923   35.067389        0.020514          1.124722        0.024097                    468.0
       S3 RRTStarSmart      20.0    5.0          0.25     778.637923   35.067389        0.020514          1.124722        0.023312                    539.5
       S4  APFMultiRRT      20.0   20.0          1.00     968.243626   28.103317        0.020011          0.858958        0.046703                   1571.0
       S4     MultiRRT      20.0   20.0          1.00    1011.815188   72.829258        0.031139          1.211203        0.014241                    800.5
       S4  RALMultiRRT      20.0   20.0          1.00     955.800974   17.105195        0.020375          1.175656        0.310093                   5037.0
       S4      RRTStar      20.0   20.0          1.00    1019.045277   68.819733        0.028538          1.819731        0.019113                    953.5
       S4 RRTStarSmart      20.0   20.0          1.00    1014.005673   63.411523        0.027533          2.276706        0.018517                   1157.5
```

## Independent random-map summary
```
map_group    algorithm  attempts  valid  success_rate  median_length  iqr_length  median_turning  median_clearance  median_runtime  median_collision_checks
       D1     MultiRRT      15.0   15.0      1.000000     986.353153   38.237469        0.011300          3.071073        0.010716                    699.0
       D1      RRTStar      15.0   15.0      1.000000     986.353153   21.764288        0.014187          2.908642        0.013137                    784.0
       D1 RRTStarSmart      15.0   15.0      1.000000     986.353153   21.764288        0.014187          2.908642        0.015088                    971.0
       D1  APFMultiRRT      15.0   15.0      1.000000     999.861880   29.187759        0.016068          1.620681        0.048454                   1614.0
       D1  RALMultiRRT      15.0   15.0      1.000000     990.547880   24.317523        0.016641          2.121916        0.364639                   6339.0
       D2     MultiRRT      15.0   13.0      0.866667    1051.798528   46.872460        0.024783          1.527591        0.020884                    801.0
       D2      RRTStar      15.0   13.0      0.866667    1032.670582   45.381711        0.019741          1.567521        0.026637                    923.0
       D2 RRTStarSmart      15.0   13.0      0.866667    1032.670582   45.381711        0.019741          1.567521        0.029019                   1075.0
       D2  APFMultiRRT      15.0   15.0      1.000000    1016.165585   58.893920        0.021003          1.387278        0.076389                   1743.0
       D2  RALMultiRRT      15.0   15.0      1.000000    1002.221352   37.203667        0.021763          0.954482        0.482620                   5933.0
       D3     MultiRRT      15.0   11.0      0.733333    1058.413208   70.892953        0.036667          1.067653        0.034062                    921.0
       D3      RRTStar      15.0   10.0      0.666667    1051.585619   72.178712        0.030796          1.041461        0.039101                   1048.0
       D3 RRTStarSmart      15.0   11.0      0.733333    1055.509899   65.027050        0.028754          0.923792        0.047981                   1224.0
       D3  APFMultiRRT      15.0   15.0      1.000000    1003.577087   29.908858        0.028708          0.850979        0.094431                   1658.0
       D3  RALMultiRRT      15.0   15.0      1.000000     992.465984   16.905171        0.021381          0.667069        0.530289                   5525.0
   Narrow     MultiRRT      15.0    4.0      0.266667    1125.437186   16.405593        0.017161          1.175949        0.014625                    605.0
   Narrow      RRTStar      15.0    4.0      0.266667    1122.639789   22.269687        0.022776          1.324144        0.021940                    808.0
   Narrow RRTStarSmart      15.0    4.0      0.266667    1122.466077   22.900007        0.020545          1.324144        0.021193                    919.0
   Narrow  APFMultiRRT      15.0    1.0      0.066667    1114.738260    0.000000        0.020933          1.009128        0.297572                   7696.0
   Narrow  RALMultiRRT      15.0    9.0      0.600000    1124.415019   20.594372        0.018666          1.352849        1.089967                  17990.0
```

## APF versus RAL paired deltas
Positive `delta_runtime` means RAL is slower. Positive `delta_length` means RAL produced a longer path. These are descriptive effects rather than assumptions of universal superiority.
```
experiment          metric  n  mean_delta  median_delta  bootstrap_mean_ci_low  bootstrap_mean_ci_high
     fixed    delta_length 74    1.662343     -2.328030              -8.481656               12.320890
     fixed   delta_turning 74    0.002814      0.001097               0.000100                0.005567
     fixed delta_clearance 74    0.048191      0.185060              -0.367013                0.509798
     fixed   delta_runtime 80    0.537776      0.455475               0.460333                0.622614
    random    delta_length 46  -10.681420    -10.440534             -18.521696               -2.516611
    random   delta_turning 46    0.000991     -0.000020              -0.003094                0.005348
    random delta_clearance 46   -0.362760     -0.155316              -0.915892                0.192727
    random   delta_runtime 60    0.506178      0.403489               0.442044                0.580028
```

## Reproduction
Run `python reproduce_all.py`. It executes tests, regenerates experiment CSVs, tables, figures, and this report.