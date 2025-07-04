# ESP32-S3 压力传感器数据采集系统

一个基于ESP32-S3的高精度压力信号实时采集和WiFi传输系统，支持多通道ADC采集、数据滤波、时间同步和TCP网络传输。

## 🚀 主要功能

### 数据采集
- **多通道ADC采集**：支持7路压力传感器信号同时采集
- **高采样率**：10ms定时器触发，实现100Hz采样频率
- **数据滤波**：集成滑动窗口算法，有效滤除噪声
- **硬件校准**：支持ADC硬件校准，提高测量精度

### 网络通信
- **WiFi连接**：STA模式连接局域网WiFi
- **TCP客户端**：建立与远程服务器的TCP连接
- **实时传输**：将压力数据实时发送到指定服务器
- **断线重连**：自动检测连接状态并重新建立连接

### 时间同步
- **网络时间同步**：接收服务器时间戳进行时间校准
- **高精度时间戳**：毫秒级时间戳标记每次采样数据
- **时间格式化**：标准时间格式输出（HH:MM:SS.mmm）

### 显示和反馈
- **LCD显示**：实时显示系统状态和网络连接信息
- **LED指示**：系统运行状态指示
- **错误处理**：完善的错误检测和处理机制

## 🛠️ 硬件要求

### 主控板
- **ESP32-S3开发板**（正点原子ESP32-S3开发板）
- **WiFi模块**：ESP32-S3内置WiFi功能
- **ADC通道**：使用ADC1的通道2-8（GPIO 3-9）

### 外设模块
- **LCD显示屏**：0.96寸SPI接口LCD（ST7735驱动）
- **压力传感器**：7路模拟电压输出型压力传感器
- **LED指示灯**：系统状态指示LED

### 引脚配置
```
ADC输入引脚:
- 通道1: GPIO3  (ADC1_CHANNEL_2)
- 通道2: GPIO4  (ADC1_CHANNEL_3) 
- 通道3: GPIO5  (ADC1_CHANNEL_4)
- 通道4: GPIO6  (ADC1_CHANNEL_5)
- 通道5: GPIO7  (ADC1_CHANNEL_6)
- 通道6: GPIO8  (ADC1_CHANNEL_7)
- 通道7: GPIO9  (ADC1_CHANNEL_8)

SPI接口:
- MOSI: GPIO11
- SCLK: GPIO12
- MISO: GPIO13

LCD控制:
- CS:   GPIO39
- RST:  GPIO38
- WR:   GPIO40
- BL:   GPIO41

LED指示:
- LED:  GPIO1
```

## 📁 项目结构

```
WiFi_TCPServer_2tasks/
├── CMakeLists.txt              # 项目构建配置
├── sdkconfig                   # ESP-IDF SDK配置
├── partitions-16MiB.csv        # 分区表配置
├── README.md                   # 项目说明文档
├── main/                       # 主程序目录
│   ├── CMakeLists.txt
│   ├── main.c                  # 主程序入口
│   └── APP/                    # 应用程序模块
│       ├── wifi_config.h       # WiFi配置头文件
│       └── wifi_config.c       # WiFi配置实现
├── components/                 # 自定义组件
│   └── BSP/                    # 板级支持包
│       ├── CMakeLists.txt
│       ├── ADC/                # ADC驱动
│       │   ├── adc1.h
│       │   └── adc1.c
│       ├── LCD/                # LCD显示驱动
│       │   ├── lcd.h
│       │   ├── lcd.c
│       │   └── lcdfont.h       # LCD字体库
│       ├── LED/                # LED控制驱动
│       │   ├── led.h
│       │   └── led.c
│       ├── SPI/                # SPI通信驱动
│       │   ├── spi.h
│       │   └── spi.c
│       └── KEY/                # 按键驱动
│           ├── key.h
│           └── key.c
└── build/                      # 编译输出目录
    ├── WiFi_TCPServer_2tasks.bin
    ├── WiFi_TCPServer_2tasks.elf
    └── ...
```

## ⚙️ 系统配置

### 网络配置
```c
#define REMOTE_IP_ADDR "192.168.1.100"  // 远程服务器IP地址
#define LWIP_DEMO_PORT 8081              // TCP连接端口
```

### 采集配置
```c
#define TIMER_INTERVAL 10                // ADC采集间隔（毫秒）
#define ADC_CHANNEL_COUNT 7             // ADC通道数量
#define SLIDING_WINDOW_SIZE 2           // 滑动窗口大小
```

### WiFi配置
在 `main/APP/wifi_config.c` 中配置WiFi连接参数：
- SSID（网络名称）
- Password（网络密码）

## 🔧 编译和烧录

### 环境要求
- **ESP-IDF v5.4.0+**
- **CMake 3.5+**
- **Python 3.6+**

### 编译步骤
1. 设置ESP-IDF环境变量
2. 配置目标芯片：`idf.py set-target esp32s3`
3. 编译项目：`idf.py build`
4. 烧录固件：`idf.py flash`
5. 监视输出：`idf.py monitor`

### 使用VS Code任务
项目已配置VS Code任务，可直接使用：
- **Build**: 编译项目
- **Flash**: 烧录到设备
- **Monitor**: 串口监视器
- **Clean**: 清理编译文件

## 🌐 网络接口

### 数据格式
系统通过TCP连接发送格式化的压力数据：
```
HH:MM:SS.mmm  CH1_mV CH2_mV CH3_mV CH4_mV CH5_mV CH6_mV CH7_mV
```
示例：
```
14:30:25.123  1250 1180 1320 1050 1400 1290 1150
```

### 时间同步协议
系统启动时从服务器接收时间同步数据：
```
格式: HH:MM:SS.mmm
示例: 14:30:25.123
```

### 服务器端要求
- 支持TCP服务器功能
- 连接建立后首先发送时间同步数据
- 持续接收压力传感器数据流

## 🔄 工作流程

### 系统初始化
1. **硬件初始化**：GPIO、SPI、ADC、LCD等硬件模块初始化
2. **WiFi连接**：连接配置的WiFi网络
3. **TCP连接**：建立与远程服务器的TCP连接
4. **时间同步**：接收服务器时间戳并校准本地时间
5. **任务创建**：创建数据采集和处理任务

### 数据采集流程
1. **定时触发**：10ms定时器触发ADC采集
2. **多通道采集**：同时读取7个ADC通道数据
3. **滤波处理**：使用滑动窗口算法进行数据滤波
4. **队列传输**：将处理后的数据发送到处理队列

### 数据处理流程
1. **队列接收**：从采集队列接收ADC数据
2. **时间戳计算**：计算精确的时间戳
3. **单位转换**：将ADC原始值转换为电压值（mV）
4. **数据格式化**：按协议格式组织数据
5. **网络发送**：通过TCP连接发送到服务器

### 错误处理
- **网络断线检测**：自动检测TCP连接状态
- **重连机制**：连接断开时自动重新建立连接
- **数据校验**：确保数据完整性和准确性
- **状态显示**：LCD实时显示系统状态

## 📊 性能指标

- **采样频率**：100Hz（10ms间隔）
- **ADC精度**：12位分辨率
- **电压测量范围**：0-3.3V（11dB衰减）
- **时间精度**：毫秒级时间戳
- **数据传输延迟**：< 50ms
- **多任务处理**：FreeRTOS实时调度

## 🔍 调试和监控

### 串口监控
通过ESP-IDF Monitor查看系统运行日志：
```bash
idf.py monitor
```

### LCD状态显示
- 网络连接状态
- TCP连接状态  
- 数据发送状态
- ADC采集值显示

### LED指示
- 系统正常运行指示
- 错误状态指示

## 📝 开发说明

### 扩展功能
- 可增加更多ADC通道
- 支持不同类型的传感器
- 可配置不同的采样频率
- 支持数据存储到SD卡

### 自定义配置
- 修改网络参数：`main.c`中的网络配置
- 调整采集参数：修改定时器间隔和通道数量
- 更改显示内容：修改LCD显示函数
