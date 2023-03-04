# LIS3DH (3-axis accelerometer): STM32 CubeMX HAL-based library.

A bare-bones library for interacting with the LIS3DH accelerometer using STM32 HAL in CubeMX.

I've tested it with my [LIS3DH breakout board](https://github.com/jedp/LIS3DH-breakout) connected
to my [STM32F04 debug board](https://github.com/jedp/STM32F042-board-v2) via I2C.

I have not yet needed the SPI interface, so that is *not yet implemented* in this library!

Place the `.h` and `.c` files in your `Core/Inc` and `Core/Src`.

1. Include the library.

```
/* USER CODE BEGIN Includes */
#include "lis3dh.h"
/* USER CODE END Includes */
```

2. Allocate storage.

```
/* USER CODE BEGIN PV */

// Allocate a buffer for reading data from the sensor.
// Six bytes required to read XYZ data.
uint8_t xyz_buf[6] = { 0 };

// New instance of the lis3dh convenience object.
lis3dh_t lis3dh;

// lis3dh calls return this HAL status type.
HAL_StatusTypeDef status;
/* USER CODE END PV */
```

3. Initialize the device.

Assumes you have configured your project to use I2C. This will generate `I2C_HandleTypeDef hi2c1;`.

The `init` function is not very clever. It only does things I've so far had a use for.

One important detail is that there's no way to configure the alternate I2C address (0x19).
So definitely fix that if you need it.

```
  /* USER CODE BEGIN 2 */

  status = lis3dh_init(&lis3dh, &hi2c1, xyz_buf, 6);

  if (status != HAL_OK) {
    // Unable to communicate with device!
  }

  /* USER CODE END 2 */
```

4. Read data.

Inside your main loop:

```
    if (lis3dh_xyz_available(&lis3dh)) {
      status = lis3dh_get_xyz(&lis3dh);
      // You now have raw acceleration of gravity in lis3dh->x, y, and z.
    }
```

