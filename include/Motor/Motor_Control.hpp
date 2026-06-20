#pragma once
#include "Ethercat/ethercat_master.hpp"
#include "Ethercat/x5s_axis.hpp"
#include "Ethercat/config.hpp"
#include <vector>
#include <array>
#include <cstdint>


class MotorControl {

public:
    MotorControl(EthercatMaster& master,std::vector<X5sAxis>& axes);

    //基础函数：
    X5sAxis& Axis(int axis_index); // 获取指定轴对象

    EthercatMaster& Master();

    int AxisCount() const;

    void ReadAllAxes();     //读取所有轴

    void WriteAllAxes();        //写入所有轴

    void SyncClocks();          //同步时钟

    // 等待使能所有轴
    bool WaitAllAxesOperationEnabled(
        int required_ready_cycles,
        int timeout_cycles
    );

    //等待所选轴使能
    bool WaitAxesOperationEnabled(const std::vector<int>& axis_indices,int required_ready_cycles,int timeout_cycles);

    //保持所有轴的位置
    void HoldAllAxesCurrentPosition(
        int hold_cycles
    );

    //保持所选轴位置
    void HoldAxesCurrentPosition(const std::vector<int>& axis_indices,int hold_cycles);

    //读取所选轴的当前编码器位置
    int32_t GetAxisEncoderCount(int axis_index);
    
public:

    //读取EncoderCounts接口
    const std::array<int32_t, config::kMaxAxisCount>& HomeEncoderCounts() const;

    //读取Rho接口
    const std::array<double, config::kMaxAxisCount>& HomeRhoMm() const;



private:
    EthercatMaster& master_;
    std::vector<X5sAxis>& axes_;


private:
    std::array<double, config::kMaxAxisCount> home_rho_mm_{
        0.0,60.0,120.0,                         //轴0-2
        1240.0, 1300.0, 1360.0,                  //轴3-5
        0.0,60.0,120.0,                         //轴6-8
        1240.0, 1300.0, 1360.0                  //轴9-11
        };

    std::array<int32_t, config::kMaxAxisCount> home_encoder_counts_{
        -753744,-2785110,-5492992,                  //轴0-2
        5380474, 2516466, 2041423,                  //轴3-5
        2647456,11864728,13589079,                   //轴6-8
        -2974453, -2098997, -1084057                 //轴9-11
        };

    std::array<int32_t, config::kMaxAxisCount> initialpos_encoder_counts_{
        -50388,9304275,7861681,                      //轴0-2
        1704847,-277881,-13281,                       //轴3-5
        1944435,-225004,234929,                      //轴6-8
        710673,701761,975573                         //轴9-11
        };

};