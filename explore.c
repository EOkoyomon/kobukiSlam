// Robot Template app
//
// Framework for creating applications that control the Kobuki robot

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "app_error.h"
#include "app_timer.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_spi.h"

#include "buckler.h"
#include "display.h"
#include "kobukiActuator.h"
#include "kobukiSensorPoll.h"
#include "kobukiSensorTypes.h"
#include "kobukiUtilities.h"

#include <Python.h>

// I2C manager
NRF_TWI_MNGR_DEF(twi_mngr_instance, 5, 0);

typedef enum {
  OFF,
  DRIVE_STRAIGHT,
  ROTATING,
  ROTATE_LEFT,
  ROTATE_RIGHT, 
  APPROACH, 
  RETURN
} robot_state_t;

typedef struct route 
{
  float distance;
  float rotate_angle;
  struct route *next;
} route_t;

static float measure_distance(uint16_t current_encoder, uint16_t previous_encoder) {
	const float CONVERSION = 0.00008529;
	float result;

	if (previous_encoder < current_encoder) {
		result = CONVERSION * (0xFFFF - current_encoder + previous_encoder);
	} else {
		result = (previous_encoder - current_encoder) * CONVERSION;
	}
	
	if (result > 5) {
		return 0;
	}
	return result;
}

static float measure_distance_reverse(uint16_t current_encoder, uint16_t previous_encoder) {
	const float CONVERSION = 0.00008529;

	float result;

	if (previous_encoder > current_encoder) {
		result = CONVERSION * (0xFFFF - previous_encoder + current_encoder);
	} else {
		result = (current_encoder - previous_encoder) * CONVERSION;
	}
	
	if (result > 5) {
		return 0;
	}
	return result;
}

PyObject* myModuleString = PyString_FromString((char*)"track_yellow");
PyObject* myModule = PyImport_Import(myModuleString);
PyObject* detect_left = PyObject_GetAttrString(myModule,(char*)"duck_detect_left");
PyObject* detect_right = PyObject_GetAttrString(myModule,(char*)"duck_detect_right");
PyObject* centered = PyObject_GetAttrString(myModule,(char*)"duck_centered");
PyObject* distance = PyObject_GetAttrString(myModule,(char*)"duck_distance");


static bool duck_detect_left(char *img_path) {
  PyObject* py_msg = PyString_FromString(img_path);
  PyObject* args = PyTuple_Pack(1,py_msg);
  PyObject* myResult = PyObject_CallObject(detect_left, args)
  double result = PyFloat_AsDouble(myResult);
}

static bool duck_detect_right(char *img_path) {
  PyObject* py_msg = PyString_FromString(img_path);
  PyObject* args = PyTuple_Pack(1,py_msg);
  PyObject* myResult = PyObject_CallObject(detect_right, args)
  double result = PyFloat_AsDouble(myResult);
}

static bool duck_centered(char *img_path) {
  PyObject* py_msg = PyString_FromString(img_path);
  PyObject* args = PyTuple_Pack(1,py_msg);
  PyObject* myResult = PyObject_CallObject(centered, args)
  double result = PyFloat_AsDouble(myResult);
}

static float duck_dist(char *img_path) {
  // once the duck is centered, get the distance
  PyObject* py_msg = PyString_FromString(img_path);
  PyObject* args = PyTuple_Pack(1,py_msg);
  PyObject* myResult = PyObject_CallObject(distance, args)
  double result = PyFloat_AsDouble(myResult);
 
}

static route_t * get_return_directions(void) {

}

int main(void) {

  ret_code_t error_code = NRF_SUCCESS;

  // initialize RTT library
  error_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(error_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  printf("Log initialized!\n");

  // initialize LEDs
  nrf_gpio_pin_dir_set(23, NRF_GPIO_PIN_DIR_OUTPUT);
  nrf_gpio_pin_dir_set(24, NRF_GPIO_PIN_DIR_OUTPUT);
  nrf_gpio_pin_dir_set(25, NRF_GPIO_PIN_DIR_OUTPUT);

  // initialize display
  nrf_drv_spi_t spi_instance = NRF_DRV_SPI_INSTANCE(1);
  nrf_drv_spi_config_t spi_config = {
    .sck_pin = BUCKLER_LCD_SCLK,
    .mosi_pin = BUCKLER_LCD_MOSI,
    .miso_pin = BUCKLER_LCD_MISO,
    .ss_pin = BUCKLER_LCD_CS,
    .irq_priority = NRFX_SPI_DEFAULT_CONFIG_IRQ_PRIORITY,
    .orc = 0,
    .frequency = NRF_DRV_SPI_FREQ_4M,
    .mode = NRF_DRV_SPI_MODE_2,
    .bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST
  };
  error_code = nrf_drv_spi_init(&spi_instance, &spi_config, NULL, NULL);
  APP_ERROR_CHECK(error_code);
  display_init(&spi_instance);
  display_write("Hello, Human!", DISPLAY_LINE_0);
  printf("Display initialized!\n");

  // initialize i2c master (two wire interface)
  nrf_drv_twi_config_t i2c_config = NRF_DRV_TWI_DEFAULT_CONFIG;
  i2c_config.scl = BUCKLER_SENSORS_SCL;
  i2c_config.sda = BUCKLER_SENSORS_SDA;
  i2c_config.frequency = NRF_TWIM_FREQ_100K;
  error_code = nrf_twi_mngr_init(&twi_mngr_instance, &i2c_config);
  APP_ERROR_CHECK(error_code);
  mpu9250_init(&twi_mngr_instance);
  printf("IMU initialized!\n");

  // initialize Kobuki
  kobukiInit();
  printf("Kobuki initialized!\n");

  // configure initial state
  robot_state_t state = OFF;
  KobukiSensors_t sensors = {0};

  float rotate_angle = 0;
  float distance_traveled = 0;
  uint16_t old_encoder = 0;
  uint16_t new_encoder = 0;
  route_t * directions;

  // loop forever, running state machine
  while (1) {

    // read sensors from robot
    kobukiSensorPoll(&sensors);

    // delay before continuing
    // Note: removing this delay will make responses quicker, but will result
    //  in printf's in this loop breaking JTAG
    nrf_delay_ms(100);

    // handle states
    switch(state) {

      case OFF: {
        // transition logic
        if (is_button_pressed(&sensors)) {
          state = DRIVE_STRAIGHT;
          new_encoder = sensors.leftWheelEncoder;
        } else {
          // perform state-specific actions here
          kobukiDriveDirect(0, 0);
          state = OFF;
        }
        break; // each case needs to end with break!
      }

      case DRIVE_STRAIGHT: {

        if (is_button_pressed(&sensors) ) {
          state = OFF;

        } else if (sensors.bumps_wheelDrops.bumpCenter || sensors.bumps_wheelDrops.bumpLeft || sensors.bumps_wheelDrops.bumpRight) {
          state = ROTATING;
          kobukiDriveDirect(0, 0);
          total_rotated = 0;

        } else if (duck_detect_left()) {
          state = ROTATE_LEFT;
          kobukiDriveDirect(0, 0);
          total_rotated = 0;

        } else if (duck_detect_right()) {
          state = ROTATE_RIGHT;
          kobukiDriveDirect(0, 0);
          total_rotated = 0;

        } else {
          state = DRIVE_STRAIGHT;
          kobukiDriveDirect(100, 100);
        }

        break;
      }

      case ROTATING: {

        if (is_button_pressed(&sensors)) {
          state = OFF;

        } else if (duck_detect_left()) {
          state = ROTATE_LEFT;
          kobukiDriveDirect(0, 0);
          total_rotated = 0;

        } else if (duck_detect_right()) {
          state = ROTATE_RIGHT;
          kobukiDriveDirect(0, 0);
          total_rotated = 0;

        } else if (total_rotated < 90){
          total_rotated = ;
          kobukiDriveDirect(50, -50);

        } else {
          state = DRIVE_STRAIGHT;
        }

        break;
      }


      case ROTATE_LEFT: {
        // transition logic
        
        if (is_button_pressed(&sensors) ) {
          state = OFF;

        } else if (duck_centered()) {
        	state = APPROACH;

        } else {
          kobukiDriveDirect(-10, 10);
          state = ROTATE_LEFT;
        }

        break; // each case needs to end with break!
      }

      case ROTATE_RIGHT: {
        // transition logic
        
        if (is_button_pressed(&sensors) ) {
          state = OFF;

        } else if (duck_centered()) {
          state = APPROACH;

        } else {
          kobukiDriveDirect(10, -10);
          state = ROTATE_RIGHT;
        }

        break; // each case needs to end with break!
      }

      case APPROACH: {
        if (is_button_pressed(&sensors)) {
          state = OFF;
        } else if (duck_dist <= 0.1) {
          state = RETURN;
          directions = get_return_directions();
          distance_traveled = 0;
          total_rotated = 0;
        } else {
          kobukiDriveDirect(50, 50);
          state = APPROACH;
        }
      }

      case RETURN: {
        if (is_button_pressed(&sensors)) {
          state = OFF;
        } else if (directions == NULL) {
          state = OFF;
        } else {

          if (abs(directions->distance - distance_traveled) >= 0.1) {
            kobukiDriveDirect(50, 50);
            old_encoder = new_encoder;
            new_encoder = sensors.leftWheelEncoder;
            distance_traveled += measure_distance(old_encoder, new_encoder);
          } else if (abs(directions->rotate_angle - total_rotated) >= 0.1) {
            kobukiDriveDirect(10, -10);
            total_rotated = ;
          } else {
            directions = directions->next;
          }

          state = RETURN;
        }
      }
      


      // add other cases here

    }
  }
}

