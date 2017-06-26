# Extended Kalman Filter Project
Self-Driving Car Engineer Nanodegree Program

[image1]: ./screenshot.png "Center"

This project implements a Kalman filter in C++ to predict and track the position and velocity of a moving object.  The goal is to make use of both lidar and radar data.  The algorithm makes a prediction based on previous measurements and updates the known state.  The predictions become more accurate over time as measured by the root mean squared error (RMSE).

The RSME for the given dataset is [0.09, 0.09, 0.45, 0.44], within the acceptable constraints.  See the screenshot for the final result.

![alt text][image1]
