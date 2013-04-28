#pragma once

#include "TrivisioConfig.h"

#if WIN32
#	include "pstdint.h"
#else
#	include <stdint.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

/*! \brief Trivisio sensor structure
 *
 *  The fields of the TrivisioSensor describes the senor and how to
 *  access it.
 */
struct TrivisioSensor
{
	char dev[256];           /*!< Device name where found */
	char ID[8];              /*!< Unique unit ID */
	/* \brief Sensor type */
	enum SensorType {
		COLIBRI                /*!< Colibri IMU */
	} type;
	uint16_t FWver;          /*!< Firmware version */
	uint16_t FWsubver;       /*!< Firmware subversion */
};


/* \brief Colibri boresighting types
 *
 * The boresighting modes that the Colibri supports.
 */
enum ColibriBoresightType {
	COLIBRI_HEADING_RESET = 1,   /*! Heading reset */
	COLIBRI_OBJECT_RESET = 2,    /*! Object reset */
	COLIBRI_ALIGNMENT_RESET = 3  /*! Alignment reset */
};


/*! \brief Colibri configuration structure
 *
 *  The fields of the ColibriConfig describes the setting of the Colibri.
 */
struct ColibriConfig
{
	/*! \brief Sensor constants */
	enum Sensor {
		MAG_X = 0x0001,         /*!< Magnetometer x-axis*/
		MAG_Y = 0x0002,         /*!< Magnetometer y-axis */
		MAG_Z = 0x0004,         /*!< Magnetometer z-axis */
		ACC_X = 0x0008,         /*!< Accelerometer x-axis */
		ACC_Y = 0x0010,         /*!< Accelerometer y-axis */
		ACC_Z = 0x0020,         /*!< Accelerometer z-axis */
		GYR_X = 0x0040,         /*!< Gyroscope x-axis */
		GYR_Y = 0x0080,         /*!< Gyroscope y-axis */
		GYR_Z = 0x0100,         /*!< Gyroscope z-axis */
		TEMP  = 0x0200,         /*!< Chip temperature */
		NAV   = 0x0400          /*!< Navigation (experimental, undefined result on usage) */
	} sensor;
	uint8_t magDiv;           /*!< Magnetometer divisor 2^(5+magDiv) */
	uint16_t freq;            /*!< Sampling frequency */
	uint8_t ascii:1;          /*!< ASCII output: on/off */
	uint8_t autoStart:1;      /*!< Sensor auto-start on turn on */
	uint8_t raw:1;            /*!< Raw data switch (raw/calibrated) */
};


/*! \brief TrivisioIMUData
 *
 *  This structure is used to return data from the IMU.
 */
struct TrivisioIMUData
{
	uint32_t t;               /*!< Time [0.1ms] */
	float mag_x;              /*!< Magnetometer reading, x-axis */
	float mag_y;              /*!< Magnetometer reading, y-axis */
	float mag_z;              /*!< Magnetometer reading, z-axis */
	float acc_x;              /*!< Accelerometer reading, x-axis */
	float acc_y;              /*!< Accelerometer reading, y-axis */
	float acc_z;              /*!< Accelerometer reading, z-axis */
	float gyr_x;              /*!< Gyroscope reading, x-axis */
	float gyr_y;              /*!< Gyroscope reading, y-axis */
	float gyr_z;              /*!< Gyroscope reading, z-axis */
	float temp;               /*!< Temperature reading */
	float q_w;                /*!< Orientation quaternion, w component */
	float q_x;                /*!< Orientation quaternion, x component */
	float q_y;                /*!< Orientation quaternion, y component */
	float q_z;                /*!< Orientation quaternion, z component */
};


/*! \brief TrivisioIMUCalibration
 *
 *  This structure contains the parameters used when calibrating the a
 *  Trivisio Colibri.
 */
struct TrivisioIMUCalibration
{
	float Ra[9];
	float ba[3];
	float ka[3];

	float Rg[9];
	float bg[3];
	float kg[3];
	float Kga[9];

	float Rm[9];
	float bm[3];
	float km[3];
};

#ifdef __cplusplus
}
#endif
