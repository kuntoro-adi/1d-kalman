/**
 * Simple Kalman Filter
 * Copyright © 2016 Kuntoro Adi Nugroho
 * License: MIT License 
 * This software is provided as is without any warranty.
 * 
 */
function onedkalman(measurementData, initialMeasurement, initialP, Q, R, A = 1, B = 0, u = []) {
    var numIteration = measurementData.length;

    // our estimation and covariance error
    // we add + 1 in order to take the initial value into 
    // account in computation process
    var estimation = new Array(measurementData.length + 1);
    var P = new Array(measurementData.length + 1);
    estimation[0] = initialMeasurement;
    P[0] = initialP;
    
    var KalmanGain = 0;
    
    // if the control variable / u is undefined or has inappropriate length
    if( u == undefined || u.length == 0 || u.length != measurementData.length) {
      u = new Array(measurementData.length);
      for (var j = 0; j < u.length; j++ ) u[j] = 0;   
    }
    
    for (var i = 1; i <= numIteration; i++) {
      
      // 1. measurement update / prediction
      // prior estimation
      priorEstimation = A * estimation[i - 1] + B * u[i - 1];
      // prior covariance
      priorP = P[i - 1];
     
      // 2. time update / correction
      KalmanGain = priorP / (priorP + R);
      estimation[i] = priorEstimation + (KalmanGain * (measurementData[i - 1] - priorEstimation));
      P[i] = (1 - KalmanGain) * priorP;
    }
    
    // clear the first estimation and P element because its just 
    // the initial condition
    estimation.shift();
    P.shift();
    
    return estimation;
}