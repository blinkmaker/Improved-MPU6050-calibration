# Improved MPU6050 calibration

I have made some improvements to the original MPU-6050 calibration sketch by Luis Ródenas:
1. A check has been added if the MPU-6050 is not detected, so that the sketch does not continue to find the offsets.
2. It prints out the calibration offsets in an easier format that can be simply copied and pasted in your Arduino sketch.
