#pragma once

#include "TrivisioConfig.h"
#include "TrivisioTypes.h"


#ifdef __cplusplus
extern "C" {
#endif

/*! Create new Colibri handle
 *
 * The function creates and initializes a new Colibri handle which is
 * used to interact with the physical IMU.
 *
 * \param bufLen size of the internal output buffer to be used.  An
 *        short buffer (0) ensures minimal delay until the sensor
 *        measurement is available at the risk of lost measurements.
 *        A long buffer guarantees that no data is dropped, at the
 *        same time if data is not read fast enough there is a
 *        potential risk of a bufLen*frequency before the measurement
 *        becomes available.
 *
 * return New Colibri handle
 */
Trivisio_DECLSPEC
void* colibriCreate(unsigned short bufLen);

/*! Destroy a Colibri handle
 *
 * The function destroys a Colibri handle and returns the system
 * resources allocated by it.
 *
 * \param imu The Colibri handle to destroy
 */
Trivisio_DECLSPEC
void colibriDestroy(void* imu);


/*! Open connection from Colibri handle to  hardware
 *
 * The tries to connect to and open up a connection to a Colibri IMU.
 *
 * \param imu  Colibri handle to associate with sensor.
 * \param conf Configuration to open the sensor with.  Default
 *             obtained a values (obtained with conf=0) is to only
 *             turn the sensor silent.
 * \param dev  Device to associate the handle with. If 0, the functions
 *             looks for a sensor amongst available COM<N> (Windows) and
 *             /dev/ttyACM<N> (Linux).
 *
 * \return 0 on success, otherwise non-zero.
 */
Trivisio_DECLSPEC
int colibriOpen(void* imu, const struct ColibriConfig* conf, const char* dev);

/*! Close connection between Colibri handle and hardware
 *
 * Close the connection established between given handle and IMU
 * hardware.
 *
 * \param imu The affected Colibri handle
 */
Trivisio_DECLSPEC
void colibriClose(void* imu);


/*! Start measurements
 *
 * Starts measurements from Colibri using the current configuration.
 *
 * \param imu Handle to Colibri to start
 *
 * \return 1 on success.
 */
Trivisio_DECLSPEC
int colibriStart(void* imu);

/*! Stop measurements
 *
 * Stop measurements with Colibri.
 *
 * \param imu Handle to Colibri to stop
 */
Trivisio_DECLSPEC
void colibriStop(void* imu);


/*! Get sensor ID
 *
 * Retrieves the unique Colibri ID.
 *
 * \param imu Handle to Colibri
 * \param ID  Pointer to a char* to where the ID should be written.
 *            Supplied string must be at least 7 chars long.
 */
Trivisio_DECLSPEC
void colibriGetID(void* imu, char* ID);

/*! Get sensor data
 *
 * Retrieves a sensor reading from the Colibri.  The function always
 * returns a measurement!  Either the last sensor reading available
 * from the hardware, or a default empty value.  To determine if the
 * returned data is new, check the time stamp that should be
 * monotonically increasing.  The reading from a stopped sensor has
 * the time set to 0.
 *
 * \param imu  Handle to Colibri
 * \param data Read data, see discussion above for details
 */
Trivisio_DECLSPEC
void colibriGetData(void* imu, struct TrivisioIMUData* data);


/*! Get Colibri configuration
 *
 * Retrieves the current sensor configuration from a Colibri.  Only
 * use this function when the sensor is not running.
 *
 * \param imu    Handle to Colibri
 * \param config Current sensor configuration
 */
Trivisio_DECLSPEC
void colibriGetConfig(void* imu, struct ColibriConfig* config);

/*! Set Colibri configuration
 *
 * Set the current sensor configuration.  Only use this function when
 * the sensor is not running.
 *
 * \param imu    Handle to Colibri
 * \param config Configuration to load into the sensor, if config=0
 *               apply the default configuration: activating all
 *               sensors (but navigation, sensor=1023), magDiv=3,
 *               freq=100, ascii=false, autoStart=1, and raw=0.
 */
Trivisio_DECLSPEC
void colibriSetConfig(void* imu, const struct ColibriConfig* config);


/*! Get status of F/W Kalman filtering of accelerometer data
 *
 * \param imu Handle to Colibri.
 *
 * \return Non-zero if the filter is active, otherwise zero.
 */
Trivisio_DECLSPEC
int colibriGetKaStatus(void* imu);

/*! Set status of F/W Kalman filtering of accelerometer data
 *
 * \param imu Handle to Colibri.
 * \param active Set to non-zero if the filter should be activated,
 * otherwise zero.
 */
Trivisio_DECLSPEC
void colibriSetKaStatus(void* imu, int active);

/*! Set filter gain for accelerometers
 *
 * Sets the filter gain for accelerometer measurements.  The default
 * action is no filtering.
 *
 * \param imu Colibri handle
 * \param Ka  Filter gain (3x3) row major matrix
 */
Trivisio_DECLSPEC
void colibriSetKa(void* imu, const float Ka[9]);

/*! Get filter gain for accelerometers
 *
 * Gets the filter gain for accelerometer measurements.
 *
 * \param imu Colibri handle
 * \param Ka  On return, contains filter gain (3x3) row major matrix
 */
Trivisio_DECLSPEC
void colibriGetKa(void* imu, float Ka[9]);


/*! Get status of F/W Kalman filtering of gyroscope data
 *
 * \param imu Handle to Colibri.
 *
 * \return Non-zero if the filter is active, otherwise zero.
 */
Trivisio_DECLSPEC
int colibriGetKgStatus(void* imu);

/*! Set status of F/W Kalman filtering of gyroscope data
 *
 * \param imu Handle to Colibri.
 * \param active Set to non-zero if the filter should be activated,
 * otherwise zero.
 */
Trivisio_DECLSPEC
void colibriSetKgStatus(void* imu, int active);

/*! Set filter gain for gyroscopes
 *
 * Sets the filter gain for gyroscope measurements.  The default
 * action is no filtering.
 *
 * \param imu Colibri handle
 * \param Kg  Filter gain (3x3) row major matrix
 */
Trivisio_DECLSPEC
void colibriSetKg(void* imu, const float Kg[9]);

/*! Get filter gain for gyroscopes
 *
 * Sets the filter gain for gyroscope measurements.
 *
 * \param imu Colibri handle
 * \param Kg  On return, contains filter gain (3x3) row major matrix
 */
Trivisio_DECLSPEC
void colibriGetKg(void* imu, float Kg[9]);


/*! Get status of F/W Kalman filtering of magnetometer data
 *
 * \param imu Handle to Colibri.
 *
 * \return Non-zero if the filter is active, otherwise zero.
 */
Trivisio_DECLSPEC
int colibriGetKmStatus(void* imu);

/*! Set status of F/W Kalman filtering of magnetometer data
 *
 * \param imu Handle to Colibri.
 * \param active Set to Non-zero if the filter should be activated,
 * otherwise zero.
 */
Trivisio_DECLSPEC
void colibriSetKmStatus(void* imu, int active);

/*! Set filter gain for magnetometers
 *
 * Sets the filter gain for magnetometer measurements.  The default
 * action is no filtering.
 *
 * \param imu Colibri handle
 * \param Km  Filter gain (3x3) row major matrix
 */
Trivisio_DECLSPEC
void colibriSetKm(void* imu, const float Km[9]);

/*! Get filter gain for magnetometers
 *
 * Sets the filter gain for magnetometer measurements.
 *
 * \param imu Colibri handle
 * \param Kg  On return, contains filter gain (3x3) row major matrix
 */
Trivisio_DECLSPEC
void colibriGetKg(void* imu, float Kg[9]);


/*! Get filter gain for accelerometers
 *
 * Gets the filter gain for accelerometer measurements.
 *
 * \param imu Colibri handle
 * \param Ka Contains the filter gain (3x3) row major matrix, after
 * the call
 */
Trivisio_DECLSPEC
void colibriGetKa(void* imu, float Ka[9]);

/*! Get filter gain for gyroscopes
 *
 * Gets the filter gain for gyroscope measurements.
 *
 * \param imu Colibri handle
 * \param Kg Contains the filter gain (3x3) row major matrix, after
 * the call
 */
Trivisio_DECLSPEC
void colibriGetKg(void* imu, float Kg[9]);

/*! Get filter gain for magnetometers
 *
 * Gets the filter gain for magnetometer measurements.
 *
 * \param imu Colibri handle
 * \param Km Contains the filter gain (3x3) row major matrix, after
 * the call
 */
Trivisio_DECLSPEC
void colibriGetKm(void* imu, float Km[9]);

/*! Get sensor calibration
 *
 * Retrieves the specific Colibri calibration of the selected unit.
 * \param imu Colibri handle
 * \param calib Contains the unit specific calibration data on return.
 */
Trivisio_DECLSPEC
void colibriGetCalib(void* imu, struct TrivisioIMUCalibration* calib);

/*! Set sensor calibration
 *
 * Sets the specific Colibri calibration of the selected unit.
 * \param imu Colibri handle
 * \param calib Contains the unit specific calibration data.
 *
 * \warning Overwriting the factory set calibration data, which is
 * finely tuned to every individual sensor, may a have serious impact
 * on sensor performance.  Therefore, make sure to have a copy of the
 * factory settings before overwriting and saving new calibration
 * information on the sensor.
 */
Trivisio_DECLSPEC
void colibriSetCalib(void* imu, const struct TrivisioIMUCalibration* calib);

/*! Save sensor settings permanently in sensor
 *
 * Saves the current Colibri settings permanently.
 * \param imu Colibri handle
 */
Trivisio_DECLSPEC
void colibriSaveSettings(void* imu);


/*! Get sensor boresighting status
 *
 * Gets the current boresighting setting of sensor.
 * \param imu Colibri handle
 *
 * \return Returns non-zero if boresighting is actived.
 */
Trivisio_DECLSPEC
int colibriGetBoresight(void* imu);

/*! Set sensor boresighting status
 *
 * Sets the current boresighting setting of sensor.
 * \param imu Colibri handle
 * \param active Activates boresighting if non-zero, otherwise turns
 * it off.
 *
 * \remark As of firmware version 1.407 the boresight status is saved
 * in the sensor, before that the sensor does not remember it's
 * boresight status.  Since this command should be used when the
 * sensor is running, the information is put on the sensor as it is
 * stopped.
 */
Trivisio_DECLSPEC
void colibriSetBoresight(void* imu, int active);

/*! Boresights the sensor
 *
 * Perfoms a boresight calibration of the sensor, and activates
 * boresighting.
 * \param imu Colibri handle
 * \param type Tells what type boresighting should be used in the
 * calibration.
 *
 *
 * \remark As of firmware version 1.407 the boresight status is saved
 * in the sensor, before that the sensor does not remember it's
 * boresight status.  Since this command should be used when the
 * sensor is running, the information is put on the sensor as it is
 * stopped.
 */
Trivisio_DECLSPEC
void colibriBoresight(void* imu, enum ColibriBoresightType type);


/*! Get stationary jitter reduction status for sensor
 *
 * Gets the status of the jitter reduction for stationary sensor.
 * \param imu Colibri handle
 *
 * \return 0 if jitter reduction is deactivated.
 */
Trivisio_DECLSPEC
int colibriGetJitterStatus(void* imu);


/*! Get stationary jitter reduction parameters for sensor
 *
 * Gets the parameters used for the jitter reduction.
 * \param imu Colibri handle
 * \param lambda Contains the smoothing factor on return.
 * \param th Determines how aggressive the jitter reduction is on
 * \param tol Determines the weighting of slow vs fast motion
 * return.
 *
 * \warning This function is still experimental state.  Do not relay
 * on it to behave the same way or even stay in the API between
 * different releases.
 */
Trivisio_DECLSPEC
void colibriGetJitterParam(void* imu, double* lambda, double* th, double* tol);


/* Set stationary jitter reduction status for sensor
 *
 * Sets the jitter reduction of stationary sensor.
 * \param imu Colibri handle
 * \param status If 0 turns off jitter reduction, otherwise activates
 * it
 */
Trivisio_DECLSPEC
void colibriSetJitterStatus(void* imu, int status);

/*! Set stationary jitter reduction parameters for sensor
 *
 * Sets the parameters used for the jitter reduction.
 * \param imu Colibri handle
 * \param lambda Contains the smoothing factor [0, 1].  A larger value
 * makes the jitter reduction "smoother". (Defaults to 0.9.)
 * \param th Determines how aggressive the jitter reduction is [0,
 * inf], a high value indicates aggressive jitter reduction. (Defaults
 * to 0.01.)
 * \param tol Weigthing between slow and fast motion [0, 1]. A smaller value will
 * move quicker between the two. (Defaults to 0.1)
 */
Trivisio_DECLSPEC
void colibriSetJitterParam(void* imu, double lambda, double th, double tol);


/*! Convert orientation to Euler angles
 *
 * Converts the orientation given in the measured data to Euler angles.
 *
 * \param data  The data containing the orientation information (quaternion)
 * \param euler Contains the Euler angle after the call
 */
Trivisio_DECLSPEC
void colibriEulerOri(const struct TrivisioIMUData* data, float euler[3]);


/*! Detect available sensors
 *
 * Returns a list of device names which host an available Colibri.
 *
 * \param list List of TrivisioSensor to which to write the result.
 * \param listLen How many elements there are in dev.
 *
 * \return Returns how many devices were found.  If the number of
 * devices excede size of dev as provided in listLen, the listLen
 * first devices are written to dev and -1 is returned.
 */
Trivisio_DECLSPEC
int colibriGetDeviceList(struct TrivisioSensor* list, unsigned listLen);

#ifdef __cplusplus
}
#endif
