typedef struct{
    //void setAngle(double newAngle) { angle = newAngle; }; // Used to set angle, this should be set as the starting angle
    //double getRate() { return rate; }; // Return the unbiased rate

    /* These are used to tune the Kalman filter */
    //void setQangle(double newQ_angle) { Q_angle = newQ_angle; };
    //void setQbias(double newQ_bias) { Q_bias = newQ_bias; };
   // void setRmeasure(double newR_measure) { R_measure = newR_measure; };

    //double getQangle() { return Q_angle; };
    //double getQbias() { return Q_bias; };
    //double getRmeasure() { return R_measure; };

    /* Kalman filter variables */
    float Q_angle; // Process noise variance for the accelerometer
    float Q_bias; // Process noise variance for the gyro bias
    float R_measure; // Measurement noise variance - this is actually the variance of the measurement noise

    float angle; // The angle calculated by the Kalman filter - part of the 2x1 state vector
    float bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
    float rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

    float P[2][2]; // Error covariance matrix - This is a 2x2 matrix
    float K[2]; // Kalman gain - This is a 2x1 vector
    float y; // Angle difference
    float S; // Estimate error
} Kalman;

void kalman_init(Kalman* kalman)
{
	        /* We will set the variables like so, these can also be tuned by the user */
        kalman->Q_angle = 0.001;
        kalman->Q_bias = 0.003;
        kalman->R_measure = 0.03;

        kalman->angle = 0; // Reset the angle
        kalman->bias = 0; // Reset bias

        kalman->P[0][0] = 0; // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
        kalman->P[0][1] = 0;
        kalman->P[1][0] = 0;
        kalman->P[1][1] = 0;
}

// The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
float getAngle(Kalman* kalman, float newAngle, float newRate, float dt) {
        // KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
        // Modified by Kristian Lauszus
        // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

        // Discrete Kalman filter time update equations - Time Update ("Predict")
        // Update xhat - Project the state ahead
        /* Step 1 */
        kalman->rate = newRate - kalman->bias;
        kalman->angle += dt * kalman->rate;

        // Update estimation error covariance - Project the error covariance ahead
        /* Step 2 */
        kalman->P[0][0] += dt * (dt*kalman->P[1][1] - kalman->P[0][1] - kalman->P[1][0] + kalman->Q_angle);
        kalman->P[0][1] -= dt * kalman->P[1][1];
        kalman->P[1][0] -= dt * kalman->P[1][1];
        kalman->P[1][1] += kalman->Q_bias * dt;

        // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
        // Calculate Kalman gain - Compute the Kalman gain
        /* Step 4 */
        kalman->S = kalman->P[0][0] + kalman->R_measure;
        /* Step 5 */
        kalman->K[0] = kalman->P[0][0] / kalman->S;
        kalman->K[1] = kalman->P[1][0] /kalman->S;

        // Calculate angle and bias - Update estimate with measurement zk (newAngle)
        /* Step 3 */
        kalman->y = newAngle - kalman->angle;
        /* Step 6 */
        kalman->angle += kalman->K[0] * kalman->y;
        kalman->bias += kalman->K[1] * kalman->y;

        // Calculate estimation error covariance - Update the error covariance
        /* Step 7 */
        kalman->P[0][0] -= kalman->K[0] * kalman->P[0][0];
        kalman->P[0][1] -= kalman->K[0] * kalman->P[0][1];
        kalman->P[1][0] -= kalman->K[1] * kalman->P[0][0];
        kalman->P[1][1] -= kalman->K[1] * kalman->P[0][1];

        return kalman->angle;
}
