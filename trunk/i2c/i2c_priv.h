#ifndef _I2C_PRIV_H
#define _I2C_PRIV_H

// Helper function
static __attribute__ ((always_inline)) void i2c_check_range(int i2c_id)
{
#if defined _MI2C3IF
	// I2C_1 - I2C_3 available
	ERROR_CHECK_RANGE(i2c_id, I2C_1, I2C_3, I2C_ERROR_INVALID_ID);
#elif defined _MI2C2IF
	// I2C_1 - I2C_2 available
	ERROR_CHECK_RANGE(i2c_id, I2C_1, I2C_2, I2C_ERROR_INVALID_ID);
#else
	// Only I2C_1 available
	ERROR_CHECK_RANGE(i2c_id, I2C_1, I2C_1, I2C_ERROR_INVALID_ID);
#endif
}

#endif // _I2C_PRIV_H

