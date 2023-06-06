
#pragma once
#include "Driver/ICM42688.hpp"
#include "UserApp/tool.hpp"
#include <iostream>
#include <string>

namespace deval {

class IMUService {
 public:
  typedef struct {
    uint16_t gyr[3]{};
    uint16_t acc[3]{};
    uint16_t temp{};
    uint8_t id{};
  } mtSingleDataPack;

  typedef struct {
    drvf::IMURawData imu0;
  } mtDataPack;

  IMUService():mainboard_imu_(5){
    imu0_rdy_ = false;
  }

  int Start(const bool imu1_enable = false) {
    imu1_enabled_ = imu1_enable;
    printf("ImuService: Initing...\r\n");

    printf("ImuService: Initing IMU0 as ICM42688...\r\n");
    bool has_init = false;
    int8_t error_code = 0;
    for (uint8_t i = 0; i < 3; i++) {
      error_code = mainboard_imu_.DebugInit(false);
      if (error_code == 0) {
        has_init = true;
        imu0_rdy_ = true;
        break;
      } else {
        has_init = false;
        printf("ImuService: try %d init ICM42688 failed! error code : %d \r\n", i + 1, error_code);
        HAL_Delay(1000);
      }
    }
    if (has_init) {
      printf("ImuService: Init imu0 as ICM42688 succeeded!\r\n");
    } else {
      printf("ImuService: Init imu0 as ICM42688 init failed!\r\n");
      return -1;
    }
    return 1;
  }

  void Loop() {
    uint32_t timestamp_us_last =0 ;
    uint32_t timestamp_us_snap =0 ;
    while (running_) {
      data_pack_.imu0.id = 0x00;
      data_pack_.imu0.vaild = false;
      if (imu0_rdy_ && !mainboard_imu_.ReadRaw(data_pack_.imu0)) {
        printf("mainboard_imu_ ICM42688 read error!\r\n");
      } else if(imu0_rdy_ == false){
        printf("mainboard_imu_ not ready\r\n");
        HAL_Delay(1000);
      }
      else
      {
        // read success
        data_pack_.imu0.id = 0x47;
        data_pack_.imu0.vaild = true;
        // py_f2s4printf(printf_buf,data_pack_.imu0.accel[0],sizeof(data_pack_.imu0.accel[0]));
        // printf("%s\r\n",printf_buf);
        // py_f2s4printf(printf_buf,data_pack_.imu0.timestamp_us,sizeof(data_pack_.imu0.timestamp_us));
        timestamp_us_snap = data_pack_.imu0.timestamp_us-timestamp_us_last;
        std::string int_to_string = std::to_string(timestamp_us_snap); 
        printf("read snap is %s\r\n", int_to_string.c_str());
        timestamp_us_last = data_pack_.imu0.timestamp_us;
        //printf();
      }
      //HAL_Delay(0);
    }
  }

 private:
  bool running_ = true;
  bool imu1_enabled_ = false;
  bool imu0_rdy_ = false;
  mtDataPack data_pack_{};
  drvf::ICM42688 mainboard_imu_;
  char printf_buf[250];
};

}  // namespace deval