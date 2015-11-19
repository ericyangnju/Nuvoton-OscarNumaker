#ifndef CONFIG_H
#define CONFIG_H

#define I2C_IRQ
#define SYSTICK
//#define AUDIO_REC_PLAYBACK_ENABLE
#define OLED_ENABLE //ericyang 20151104
#define BT_ENABLE  //ericyang 20151103
#define MPU6050_ENABLE //ericyang 20151104
#define HUMITURESENSOR_ENABLE //ericyang 20151105 humidity & tempriture sensor enable
#ifdef HUMITURESENSOR_ENABLE
#define HDC1000_ENABLE //ericyang 20151104
//#define DHT11_ENABLE //ericyang 20151104
#endif
#define ADCSENSOR_ENABLE
#ifdef ADCSENSOR_ENABLE
#define LIGHTSENSOR_ENABLE //ericyang 20151104
#define IRSENSOR_ENABLE //ericyang 20151104
#define GASSENSOR_ENABLE //ericyang 20151104
#endif

//#define DEBUG_ENABLE //ericyang 20151111 for use printf(),be sure: DEBUG_ENABLE_SEMIHOST enable in KEIL DEFINE
#define DEBUG_BT_WIFI_ENABLE //ericyang 20151116
#define I2C_TIMEOUT_ENABLE //ericynag 20151111
#ifdef I2C_TIMEOUT_ENABLE
#define I2C_TIMEOUT_NOVALIDACK_COUNT 0x20
#define I2C_TIMEOUT_UNREACHABLE_COUNT 0x100
#endif
//#define NUVOTON_BLE_WIFI_ENABLE  //ericyang 20151118
#ifndef NUVOTON_BLE_WIFI_ENABLE
#define WEBEE_BLE_ENABLE  //ericyang 20151112
#define LONGSIS_WK1221_ENABLE //ericyang 20151116
#endif
#endif //end of config.h
