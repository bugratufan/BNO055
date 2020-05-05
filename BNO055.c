/*
 * BNO055.c
 *
 *  Created on: 23 Oca 2018
 *      Author: bugra
 */
#include "BNO055.h"
extern I2C_HandleTypeDef hi2c1;
uint8_t mode = OPERATION_MODE_NDOF;

int8_t begin( void ){
	/*
	 * IMU Begin function
	 */
	uint8_t id = I2C_read8(BNO055_ADDRESS_A, BNO055_CHIP_ID_ADDR);
	if(id != BNO055_ID){
		HAL_Delay(20); // hold on for boot
		id = I2C_read8(BNO055_ADDRESS_A, BNO055_CHIP_ID_ADDR);
		if(id != BNO055_ID) {
			return -1;  // still not? ok bail
		}
	}
	setMode(OPERATION_MODE_CONFIG);
	I2C_write8(BNO055_ADDRESS_A, BNO055_SYS_TRIGGER_ADDR, 0x20);
	while (I2C_read8(BNO055_ADDRESS_A, BNO055_CHIP_ID_ADDR) != (int8_t)BNO055_ID){
		HAL_Delay(20);
	}
	HAL_Delay(20);
	/* Set to normal power mode */
	I2C_write8(BNO055_ADDRESS_A, BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);
	HAL_Delay(10);
	I2C_write8(BNO055_ADDRESS_A, BNO055_PAGE_ID_ADDR, 0);
	HAL_Delay(20);
	I2C_write8(BNO055_ADDRESS_A, BNO055_SYS_TRIGGER_ADDR, 0x0);
	HAL_Delay(20);
	/* Set the requested operating mode (see section 3.3) */
	setMode(mode);
	HAL_Delay(20);
	return 0;
}

void setMode(uint8_t mode){
	/*
	 * To change operation mode
	 *
	 * OPERATION MODES:
	 *
	 * OPERATION_MODE_CONFIG
	 * OPERATION_MODE_ACCONLY
	 * OPERATION_MODE_MAGONLY
	 * OPERATION_MODE_GYRONLY
	 * OPERATION_MODE_ACCMAG
	 * OPERATION_MODE_ACCGYRO
	 * OPERATION_MODE_MAGGYRO
	 * OPERATION_MODE_AMG
	 * OPERATION_MODE_IMUPLUS
	 * OPERATION_MODE_COMPASS
	 * OPERATION_MODE_M4G
	 * OPERATION_MODE_NDOF_FMC_OFF
	 * OPERATION_MODE_NDOF
	 */

	I2C_write8(BNO055_ADDRESS_A, BNO055_OPR_MODE_ADDR, mode);
	HAL_Delay(20);
}

void setExtCrystalUse(uint8_t usextal){
	/*
	 * !!!UNAPPROVED!!!
	 */
	setMode(OPERATION_MODE_CONFIG);
	HAL_Delay(25);
	I2C_write8(BNO055_ADDRESS_A, BNO055_PAGE_ID_ADDR, 0);
	if (usextal == 1) {
		I2C_write8(BNO055_ADDRESS_A, BNO055_SYS_TRIGGER_ADDR, 0x80);
	}
	else {
		I2C_write8(BNO055_ADDRESS_A, BNO055_SYS_TRIGGER_ADDR, 0x00);
	}
	HAL_Delay(10);
	/* Set the requested operating mode (see section 3.3) */
	setMode(mode);
	HAL_Delay(20);
}

int8_t getTemp(void){
	/*
	 * return uint8_t temp data
	 */
	int8_t temp = (int8_t)(I2C_read8(BNO055_ADDRESS_A, BNO055_TEMP_ADDR));
	return temp;
}

void getCalibration(uint8_t* sys, uint8_t* gyro, uint8_t* accel, uint8_t* mag){
	uint8_t calData = I2C_read8(BNO055_ADDRESS_A, BNO055_CALIB_STAT_ADDR);
	if (sys != NULL) {
		*sys = (calData >> 6) & 0x03;
	}
	if (gyro != NULL) {
		*gyro = (calData >> 4) & 0x03;
	}
	if (accel != NULL) {
		*accel = (calData >> 2) & 0x03;
	}
	if (mag != NULL) {
		*mag = calData & 0x03;
	}
}

void getSystemStatus(uint8_t *system_status, uint8_t *self_test_result, uint8_t *system_error){
	I2C_write8(BNO055_ADDRESS_A, BNO055_PAGE_ID_ADDR, 0);
	HAL_Delay(10);
	/* System Status (see section 4.3.58)
     ---------------------------------
     0 = Idle
     1 = System Error
     2 = Initializing Peripherals
     3 = System Iniitalization
     4 = Executing Self-Test
     5 = Sensor fusio algorithm running
     6 = System running without fusion algorithms */

	if (system_status != 0){
		*system_status    = I2C_read8(BNO055_ADDRESS_A, BNO055_SYS_STAT_ADDR);
		HAL_Delay(20);
	}

	/* Self Test Results (see section )
     --------------------------------
     1 = test passed, 0 = test failed

     Bit 0 = Accelerometer self test
     Bit 1 = Magnetometer self test
     Bit 2 = Gyroscope self test
     Bit 3 = MCU self test

     0x0F = all good! */

	if (self_test_result != 0){
		*self_test_result = I2C_read8(BNO055_ADDRESS_A, BNO055_SELFTEST_RESULT_ADDR);
		HAL_Delay(20);
	}

	/* System Error (see section 4.3.59)
     ---------------------------------
     0 = No error
     1 = Peripheral initialization error
     2 = System initialization error
     3 = Self test result failed
     4 = Register map value out of range
     5 = Register map address out of range
     6 = Register map write error
     7 = BNO low power mode not available for selected operat ion mode
     8 = Accelerometer power mode not available
     9 = Fusion algorithm configuration error
     A = Sensor configuration error */

	if (system_error != 0){
		*system_error     = I2C_read8(BNO055_ADDRESS_A, BNO055_SYS_ERR_ADDR);
	}
	HAL_Delay(200);
}

void getVector(uint8_t vector_type, double* xyz){
	/*
	 * xyz = The address of xyz data array ( double xyz[3] )
	 * vector types:
	 * VECTOR_ACCELEROMETER
	 * VECTOR_MAGNETOMETER
	 * VECTOR_GYROSCOPE
	 * VECTOR_EULER
	 * VECTOR_LINEARACCEL
	 * VECTOR_GRAVITY
	 */
	int16_t x, y, z;
	x = y = z = 0;

	/* Read vector data (6 bytes) */
	x = I2C_read16(BNO055_ADDRESS_A, vector_type);
	y = I2C_read16(BNO055_ADDRESS_A, vector_type+2);
	z = I2C_read16(BNO055_ADDRESS_A, vector_type+4);

	switch(vector_type){
    	case VECTOR_MAGNETOMETER:
    		/* 1uT = 16 LSB */
    		xyz[0] = ((double)x)/16.0;
    		xyz[1] = ((double)y)/16.0;
    		xyz[2] = ((double)z)/16.0;
    		break;
    	case VECTOR_GYROSCOPE:
    		/* 1dps = 16 LSB */
    		xyz[0] = ((double)x)/16.0;
    		xyz[1] = ((double)y)/16.0;
    		xyz[2] = ((double)z)/16.0;
    		break;
    	case VECTOR_EULER:
    		/* 1 degree = 16 LSB */
    		xyz[0] = ((double)x)/16.0;
    		xyz[1] = ((double)y)/16.0;
    		xyz[2] = ((double)z)/16.0;
    		break;
		case VECTOR_ACCELEROMETER:
		case VECTOR_LINEARACCEL:
		case VECTOR_GRAVITY:
			/* 1m/s^2 = 100 LSB */
			xyz[0] = ((double)x)/100.0;
			xyz[1] = ((double)y)/100.0;
			xyz[2] = ((double)z)/100.0;
			break;
	}
}
/*
 * UNDER CONSTRUCTION!!!
 * getRevInfo
 * getQuat
 * getSensor
 * getEvent
 * getSensorOffsets
 * getSensorOffsets
 * setSensorOffsets
 * setSensorOffsets
 * isFullyCalibrated
 */


int8_t I2C_read8(uint8_t slave, uint8_t address){
	/*
	 * !IMPORTANT: This function shifts the input slave address one bit to the left
	 * slave = Slave addess
	 * address = Register address
	 *
	 * return uint8_t I2C_Receive
	 */
	slave = slave<<1;
	uint8_t val=0;
	HAL_I2C_Master_Transmit(&hi2c1, slave, &address, 1, 100 );
	HAL_Delay(20);
	HAL_I2C_Master_Receive(&hi2c1,slave,(uint8_t *)&val, 1, 100);
	return val;
}


void I2C_write8(uint8_t slave, uint8_t address, uint8_t data){
	/*
	 * !IMPORTANT: This function shifts the input slave address one bit to the left
	 * slave = Slave addess
	 * address = Register address
	 * data = Transmit data
	 *
	 * return void
	 */
	slave = slave<<1;
	static uint8_t buffer[2];
	buffer[0]=address;
	buffer[1]=data;
	HAL_I2C_Master_Transmit(&hi2c1,slave,buffer,2,100);
    HAL_Delay(20);
}

int16_t I2C_read16(uint8_t slave, uint8_t address){
	/*
	 * !IMPORTANT: This function shifts the input slave address one bit to the left
	 * slave = Slave addess
	 * address = Register address
	 *
	 * return uint16_t I2C_Receive
	 */
	slave = slave<<1;
	int16_t val=0;
	HAL_I2C_Master_Transmit(&hi2c1, slave, &address, 1, 100 );
	HAL_Delay(20);
	HAL_I2C_Master_Receive(&hi2c1,slave, (uint8_t *)&val, 2, 100);
	return val;
}

