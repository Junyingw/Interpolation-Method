==== BUILD ON MAC OS X ====

Unzip the files

> cd mocapPlayer
> cd make
>./mocapPlayer


Command line for generating .amc files:

./interpolate dance.asf dance.amc l e 2 LinearEuler_output_dance.amc
./interpolate dance.asf dance.amc l q 2 LinearQuaternion_output_dance.amc

>./interpolate dance.asf dance.amc l e 20 LinearEuler_output_dance.amc
>./interpolate dance.asf dance.amc b e 20 BezierEuler_output_dance.amc
>./interpolate dance.asf dance.amc l q 20 LinearQuaternion_output_dance.amc
>./interpolate dance.asf dance.amc b q 20 BezierQuaternion_output_dance.amc

>./interpolate martialArts.asf martialArts.amc l e 40 le_martial.amc
>./interpolate martialArts.asf martialArts.amc b e 40 be_martial.amc
>./interpolate martialArts.asf martialArts.amc l q 40 lq_martial.amc
>./interpolate martialArts.asf martialArts.amc b q 40 bq_martial.amc

===== FINISHED PARTS ======

* Bezier interpolation for Euler angles
* SLERP interpolations for Quaternions
* Bezier SLERP for Quaternions
* Motion Capture Interpolation Report (Graphs were plotted by Matlab)
* Screenshots of three videos included

Functions(interpolator.cpp):

1)Euler2Rotation (matrixMultiply)
2)BezierInterpolationEuler
3)LinearInterpolationQuaternion
4)BezierInterpolationQuaternion
5)Euler2Quaternion
6)Quaternion2Euler
7)Slerp
8)Double
9)DeCasteljauEuler
10)DeCasteljauQuaternion

Plot Graphs:

In order to get rotation data at certain frame, I added functions at interpolate.cpp.
When running the code, it would output batch of the rotation data. Then I copied and pasted them into the .txt file and plotted the graphs with Matlab.  


=== EXTRA CREDITS =========

* Add time counter in interpolate.cpp, used to calculate time of different methods 

For test cases(as the requirements):
1) dance amc file:
Linear Euler ---> Computation time is: 0.012643
Bezier Euler ---> Computation time is: 0.035987
SLERP Quaternion ---> Computation time is: 0.343010
Bezier SLERP Quaternion ---> Computation time is: 0.693341

2)martialArts.amc file:
Linear Euler ---> Computation time is: 0.040479
Bezier Euler ---> Computation time is: 0.110690
SLERP Quaternion ---> Computation time is: 1.125817
Bezier SLERP Quaternion ---> Computation time is: 2.231158

====== ENJOY =========

