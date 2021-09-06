/*

(c)Kouhei Ito

*/
/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/spi.h"
#include "lsm9ds1_reg.h"

/* Define communication interface */
#define SENSOR_BUS spi0

/* Private macro -------------------------------------------------------------*/
#define BOOT_TIME 20 //ms
#define PIN_CSAG  1
#define PIN_MISO  4
#define PIN_CSM   5
#define PIN_SCK   6
#define PIN_MOSI  7

typedef struct {
  spi_inst_t   *hbus;
  uint16_t cs_pin;
} sensbus_t;

/* Private variables ---------------------------------------------------------*/
static sensbus_t imu_bus = {SENSOR_BUS,
                            PIN_CSAG
                           };
static sensbus_t mag_bus = {SENSOR_BUS,
                            PIN_CSM
                           };

static int16_t data_raw_acceleration[3];
static int16_t data_raw_angular_rate[3];
static int16_t data_raw_magnetic_field[3];
static float acceleration_mg[3];
static float angular_rate_mdps[3];
static float magnetic_field_mgauss[3];
static lsm9ds1_id_t whoamI;
static lsm9ds1_status_t reg;
static uint8_t rst;
static uint8_t tx_buffer[1000];

/* Private functions ---------------------------------------------------------*/
/*
 *   WARNING:
 *   Functions declare in this section are defined at the end of this file
 *   and are strictly related to the hardware platform used.
 *
 */

static int32_t platform_write_imu(void *handle, uint8_t reg,
                                  const uint8_t *bufp, uint16_t len);
static int32_t platform_read_imu(void *handle, uint8_t reg,
                                 uint8_t *bufp, uint16_t len);
static int32_t platform_write_mag(void *handle, uint8_t reg,
                                  const uint8_t *bufp, uint16_t len);
static int32_t platform_read_mag(void *handle, uint8_t reg,
                                 uint8_t *bufp, uint16_t len);
static void tx_com( uint8_t *tx_buffer, uint16_t len );
static void platform_delay(uint32_t ms);
static void platform_init(void);

/*付け加えた*/
static inline void cs_select(uint16_t cs_pin);
static inline void cs_deselect(uint16_t cs_pin);

/* Main Example --------------------------------------------------------------*/
void lsm9ds1_read_data_polling(void)
{
  stmdev_ctx_t dev_ctx_imu;
  stmdev_ctx_t dev_ctx_mag;
  /* Initialize inertial sensors (IMU) driver interface */
  dev_ctx_imu.write_reg = platform_write_imu;
  dev_ctx_imu.read_reg = platform_read_imu;
  dev_ctx_imu.handle = (void *)&imu_bus;
  /* Initialize magnetic sensors driver interface */
  dev_ctx_mag.write_reg = platform_write_mag;
  dev_ctx_mag.read_reg = platform_read_mag;
  dev_ctx_mag.handle = (void *)&mag_bus;
  /* Initialize platform specific hardware */
  platform_init();
  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);
  /* Check device ID */
  lsm9ds1_dev_id_get(&dev_ctx_mag, &dev_ctx_imu, &whoamI);

  if (whoamI.imu != LSM9DS1_IMU_ID || whoamI.mag != LSM9DS1_MAG_ID) {
    while (1) {
      /* manage here device not found */
      printf("Device not found !\n");
      sleep_ms(1000);
    }
  }

  /* Restore default configuration */
  lsm9ds1_dev_reset_set(&dev_ctx_mag, &dev_ctx_imu, PROPERTY_ENABLE);

  do {
    lsm9ds1_dev_reset_get(&dev_ctx_mag, &dev_ctx_imu, &rst);
  } while (rst);

  /* Enable Block Data Update */
  lsm9ds1_block_data_update_set(&dev_ctx_mag, &dev_ctx_imu,
                                PROPERTY_ENABLE);
  /* Set full scale */
  lsm9ds1_xl_full_scale_set(&dev_ctx_imu, LSM9DS1_4g);
  lsm9ds1_gy_full_scale_set(&dev_ctx_imu, LSM9DS1_2000dps);
  lsm9ds1_mag_full_scale_set(&dev_ctx_mag, LSM9DS1_16Ga);
  /* Configure filtering chain - See datasheet for filtering chain details */
  /* Accelerometer filtering chain */
  lsm9ds1_xl_filter_aalias_bandwidth_set(&dev_ctx_imu, LSM9DS1_AUTO);
  lsm9ds1_xl_filter_lp_bandwidth_set(&dev_ctx_imu,
                                     LSM9DS1_LP_ODR_DIV_50);
  lsm9ds1_xl_filter_out_path_set(&dev_ctx_imu, LSM9DS1_LP_OUT);
  /* Gyroscope filtering chain */
  lsm9ds1_gy_filter_lp_bandwidth_set(&dev_ctx_imu,
                                     LSM9DS1_LP_ULTRA_LIGHT);
  lsm9ds1_gy_filter_hp_bandwidth_set(&dev_ctx_imu, LSM9DS1_HP_MEDIUM);
  lsm9ds1_gy_filter_out_path_set(&dev_ctx_imu,
                                 LSM9DS1_LPF1_HPF_LPF2_OUT);
  /* Set Output Data Rate / Power mode */
  lsm9ds1_imu_data_rate_set(&dev_ctx_imu, LSM9DS1_IMU_59Hz5);
  lsm9ds1_mag_data_rate_set(&dev_ctx_mag, LSM9DS1_MAG_UHP_10Hz);

  /* Read samples in polling mode (no int) */
  while (1) {
    /* Read device status register */
    lsm9ds1_dev_status_get(&dev_ctx_mag, &dev_ctx_imu, &reg);

    if ( reg.status_imu.xlda && reg.status_imu.gda ) {
      /* Read imu data */
      memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
      memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
      lsm9ds1_acceleration_raw_get(&dev_ctx_imu,
                                   data_raw_acceleration);
      lsm9ds1_angular_rate_raw_get(&dev_ctx_imu,
                                   data_raw_angular_rate);
      acceleration_mg[0] = lsm9ds1_from_fs4g_to_mg(
                             data_raw_acceleration[0]);
      acceleration_mg[1] = lsm9ds1_from_fs4g_to_mg(
                             data_raw_acceleration[1]);
      acceleration_mg[2] = lsm9ds1_from_fs4g_to_mg(
                             data_raw_acceleration[2]);
      angular_rate_mdps[0] = lsm9ds1_from_fs2000dps_to_mdps(
                               data_raw_angular_rate[0]);
      angular_rate_mdps[1] = lsm9ds1_from_fs2000dps_to_mdps(
                               data_raw_angular_rate[1]);
      angular_rate_mdps[2] = lsm9ds1_from_fs2000dps_to_mdps(
                               data_raw_angular_rate[2]);
      sprintf((char *)tx_buffer,
              "IMU - [mg]:%4.2f\t%4.2f\t%4.2f\t[mdps]:%4.2f\t%4.2f\t%4.2f\r\n",
              acceleration_mg[0], acceleration_mg[1], acceleration_mg[2],
              angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
      tx_com(tx_buffer, strlen((char const *)tx_buffer));
    }

    if ( reg.status_mag.zyxda ) {
      /* Read magnetometer data */
      memset(data_raw_magnetic_field, 0x00, 3 * sizeof(int16_t));
      lsm9ds1_magnetic_raw_get(&dev_ctx_mag, data_raw_magnetic_field);
      magnetic_field_mgauss[0] = lsm9ds1_from_fs16gauss_to_mG(
                                   data_raw_magnetic_field[0]);
      magnetic_field_mgauss[1] = lsm9ds1_from_fs16gauss_to_mG(
                                   data_raw_magnetic_field[1]);
      magnetic_field_mgauss[2] = lsm9ds1_from_fs16gauss_to_mG(
                                   data_raw_magnetic_field[2]);
      sprintf((char *)tx_buffer, "MAG - [mG]:%4.2f\t%4.2f\t%4.2f\r\n",
              magnetic_field_mgauss[0], magnetic_field_mgauss[1],
              magnetic_field_mgauss[2]);
      tx_com(tx_buffer, strlen((char const *)tx_buffer));
    }
  }
}

/*チップセレクトの関数を追加*/
static inline void cs_select(uint16_t cs_pin) {
    //printf("cspin=%d\n",cs_pin);
    asm volatile("nop \n nop \n nop");
    gpio_put(cs_pin, 0);  // Active low
    asm volatile("nop \n nop \n nop");
}

static inline void cs_deselect(uint16_t cs_pin) {
    asm volatile("nop \n nop \n nop");
    gpio_put(cs_pin, 1);
    asm volatile("nop \n nop \n nop");
}

/*
 * @brief  Write generic imu register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */

static int32_t platform_write_imu(void *handle, uint8_t reg, 
                                  const uint8_t *bufp, uint16_t len)
{
  sensbus_t *sensbus = (sensbus_t *)handle;

  //HAL_GPIO_WritePin(sensbus->cs_port, sensbus->cs_pin, GPIO_PIN_RESET);
  //HAL_SPI_Transmit(sensbus->hbus, &reg, 1, 1000);
  //HAL_SPI_Transmit(sensbus->hbus, (uint8_t*) bufp, len, 1000);
  //HAL_GPIO_WritePin(sensbus->cs_port, sensbus->cs_pin, GPIO_PIN_SET);

  cs_select(sensbus->cs_pin);
  spi_write_blocking(/*sensbus->hbus*/spi0, &reg, 1);
  spi_write_blocking(/*sensbus->hbus*/spi0, (uint8_t*) bufp, len);
  cs_deselect(sensbus->cs_pin);
  return 0;

}


/*
 * @brief  Write generic magnetometer register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write_mag(void *handle, uint8_t reg,
                                  const uint8_t *bufp, uint16_t len)
{
  sensbus_t *sensbus = (sensbus_t *)handle;
  /* Write multiple command */

  reg |= 0x40;
  //HAL_GPIO_WritePin(sensbus->cs_port, sensbus->cs_pin, GPIO_PIN_RESET);
  //HAL_SPI_Transmit(sensbus->hbus, &reg, 1, 1000);
  //HAL_SPI_Transmit(sensbus->hbus, (uint8_t*) bufp, len, 1000);
  //HAL_GPIO_WritePin(sensbus->cs_port, sensbus->cs_pin, GPIO_PIN_SET);
  //return 0;

  cs_select(sensbus->cs_pin);
  spi_write_blocking(/*sensbus->hbus*/spi0, &reg, 1);
  spi_write_blocking(/*sensbus->hbus*/spi0, (uint8_t*) bufp, len);
  cs_deselect(sensbus->cs_pin);
  return 0;
}

/*
 * @brief  Read generic imu register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read_imu(void *handle, uint8_t reg,
                                 uint8_t *bufp, uint16_t len)
{
  sensbus_t *sensbus = (sensbus_t *)handle;

  /* Read command */
  //reg |= 0x80;
  //HAL_GPIO_WritePin(sensbus->cs_port, sensbus->cs_pin, GPIO_PIN_RESET);
  //HAL_SPI_Transmit(sensbus->hbus, &reg, 1, 1000);
  //HAL_SPI_Receive(sensbus->hbus, bufp, len, 1000);
  //HAL_GPIO_WritePin(sensbus->cs_port, sensbus->cs_pin, GPIO_PIN_SET);
  //return 0;

  reg |= 0x80;
  cs_select(sensbus->cs_pin);
  spi_write_blocking(/*sensbus->hbus*/spi0, &reg, 1);
  spi_read_blocking(/*sensbus->hbus*/spi0, 0, bufp, len);
  cs_deselect(sensbus->cs_pin);
  return 0;

}

/*
 * @brief  Read generic magnetometer register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read_mag(void *handle, uint8_t reg,
                                 uint8_t *bufp, uint16_t len)
{
  sensbus_t *sensbus = (sensbus_t *)handle;

  /* Read multiple command */
  //reg |= 0xC0;
  //HAL_GPIO_WritePin(sensbus->cs_port, sensbus->cs_pin, GPIO_PIN_RESET);
  //HAL_SPI_Transmit(sensbus->hbus, &reg, 1, 1000);
  //HAL_SPI_Receive(sensbus->hbus, bufp, len, 1000);
  //HAL_GPIO_WritePin(sensbus->cs_port, sensbus->cs_pin, GPIO_PIN_SET);
  //return 0;

  reg |= 0xC0;
  cs_select(sensbus->cs_pin);
  spi_write_blocking(/*sensbus->hbus*/spi0, &reg, 1);
  spi_read_blocking(/*sensbus->hbus*/spi0, 0, bufp, len);
  cs_deselect(sensbus->cs_pin);
  return 0;

}

/*
 * @brief  Send buffer to console (platform dependent)
 *
 * @param  tx_buffer     buffer to transmit
 * @param  len           number of byte to send
 *
 */
static void tx_com(uint8_t *tx_buffer, uint16_t len)
{
  //CDC_Transmit_FS(tx_buffer, len);
  printf("%s",tx_buffer);
}

/*
 * @brief  platform specific delay (platform dependent)
 *
 * @param  ms        delay in ms
 *
 */
static void platform_delay(uint32_t ms)
{
  sleep_ms(ms);
}

/*
 * @brief  platform specific initialization (platform dependent)
 */
static void platform_init(void)
{
  /*  
  TIM3->CCR1 = PWM_3V3;
  TIM3->CCR2 = PWM_3V3;
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_Delay(1000);
  */

  stdio_init_all();
  sleep_ms(1000);

  printf("Hello, LSM9DS1! Reading raw data from registers via SPI...\n");

  // This example will use SPI0 at 0.5MHz.
  spi_init(SENSOR_BUS, 10 * 1000);
  gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
  gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
  gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

  // Chip select is active-low, so we'll initialise it to a driven-high state
  gpio_init(PIN_CSAG);
  gpio_init(PIN_CSM);
  gpio_set_dir(PIN_CSAG, GPIO_OUT);
  gpio_set_dir(PIN_CSM, GPIO_OUT);
  gpio_put(PIN_CSAG, 1);
  gpio_put(PIN_CSM, 1);
  sleep_ms(1000);

}

int main(void)
{
  lsm9ds1_read_data_polling();
  return 0;
}
