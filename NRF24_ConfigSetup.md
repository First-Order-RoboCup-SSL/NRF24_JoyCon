# NRF24 配置设置和数据解码文档
# NRF24 Configuration Setup and Data Decoding Documentation

## 📡 重要提醒 / Important Notice
**两个NRF24模块（发射端和接收端）必须使用完全相同的配置参数才能建立通信！**
**Both NRF24 modules (TX and RX) MUST use identical configuration parameters to establish communication!**

---

## 🔧 当前发射端配置 / Current Transmitter Configuration

### 硬件连接 / Hardware Connections
```
STM32F103 → NRF24L01+
------------------------
SPI2_SCK  (PB13) → SCK
SPI2_MISO (PB14) → MISO  
SPI2_MOSI (PB15) → MOSI
PB12             → CSN (Chip Select)
PA11             → CE  (Chip Enable)
PA8              → IRQ (Interrupt)
3.3V             → VCC
GND              → GND
```

### 通信参数 / Communication Parameters
```c
// 必须匹配的参数 / Parameters that MUST match:
频道 / Channel:           90 (2.490 GHz)
数据速率 / Data Rate:     1 Mbps
发射功率 / TX Power:      0 dBm (最大功率 / Maximum)
地址宽度 / Address Width: 5 bytes
发射地址 / TX Address:    {0x45, 0x55, 0x67, 0x10, 0x21}
载荷大小 / Payload Size:  10 bytes (固定 / Fixed)
CRC校验 / CRC:           启用，1字节 / Enabled, 1-byte
自动应答 / Auto ACK:      启用 / Enabled
自动重传 / Auto Retrans:  10次重试，1.25ms延迟 / 10 retries, 1.25ms delay
```

---

## 📊 TX Buffer 数据格式 / TX Buffer Data Format

### 数据结构 / Data Structure (10 bytes total)
```
Byte 位置 | 数据内容        | 数据类型    | 描述
Byte Pos | Data Content   | Data Type  | Description
---------|---------------|------------|------------------
0-1      | ADC Channel 1 | uint16_t   | 摇杆X轴 / Joystick X-axis
2-3      | ADC Channel 2 | uint16_t   | 摇杆Y轴 / Joystick Y-axis  
4-5      | ADC Channel 3 | uint16_t   | 摇杆Z轴 / Joystick Z-axis
6-7      | ADC Channel 4 | uint16_t   | 油门/其他 / Throttle/Other
8        | AUX1 State    | uint8_t    | 辅助按键1 / Auxiliary Button 1
9        | AUX2 State    | uint8_t    | 辅助按键2 / Auxiliary Button 2
```

### 数据范围 / Data Ranges
```c
ADC 通道 / ADC Channels (0-7字节 / Bytes 0-7):
- 数据类型: 16位无符号整数 / Data Type: 16-bit unsigned integer
- 数值范围: 0 - 4095 (12位ADC / 12-bit ADC)
- 字节序: 小端序 / Byte Order: Little Endian
- 中心值: 2048 (约1.65V / ~1.65V)

按键状态 / Button States (8-9字节 / Bytes 8-9):
- 数据类型: 8位无符号整数 / Data Type: 8-bit unsigned integer  
- 数值: 0 = 未按下 / Not Pressed, 1 = 按下 / Pressed
```

---

## 🔓 接收端数据解码 / Receiver Data Decoding

### C语言解码示例 / C Language Decoding Example
```c
#include <stdint.h>

// 数据结构定义 / Data Structure Definition
typedef struct {
    uint16_t adc_ch1;      // ADC通道1 / ADC Channel 1
    uint16_t adc_ch2;      // ADC通道2 / ADC Channel 2  
    uint16_t adc_ch3;      // ADC通道3 / ADC Channel 3
    uint16_t adc_ch4;      // ADC通道4 / ADC Channel 4
    uint8_t aux1_state;    // AUX1按键状态 / AUX1 Button State
    uint8_t aux2_state;    // AUX2按键状态 / AUX2 Button State
} __attribute__((packed)) RemoteControl_Data_t;

// 解码函数 / Decoding Function
void decode_remote_data(uint8_t* rx_buffer, RemoteControl_Data_t* data) {
    // 方法1: 直接内存拷贝 / Method 1: Direct memory copy
    memcpy(data, rx_buffer, sizeof(RemoteControl_Data_t));
    
    // 方法2: 逐字节解析 / Method 2: Byte-by-byte parsing
    data->adc_ch1 = (rx_buffer[1] << 8) | rx_buffer[0];
    data->adc_ch2 = (rx_buffer[3] << 8) | rx_buffer[2];
    data->adc_ch3 = (rx_buffer[5] << 8) | rx_buffer[4];
    data->adc_ch4 = (rx_buffer[7] << 8) | rx_buffer[6];
    data->aux1_state = rx_buffer[8];
    data->aux2_state = rx_buffer[9];
}

// 使用示例 / Usage Example
void process_received_data(uint8_t* rx_buffer) {
    RemoteControl_Data_t remote_data;
    decode_remote_data(rx_buffer, &remote_data);
    
    // 打印解码后的数据 / Print decoded data
    printf("ADC1: %d, ADC2: %d, ADC3: %d, ADC4: %d\n", 
           remote_data.adc_ch1, remote_data.adc_ch2, 
           remote_data.adc_ch3, remote_data.adc_ch4);
    printf("AUX1: %s, AUX2: %s\n", 
           remote_data.aux1_state ? "按下/Pressed" : "释放/Released",
           remote_data.aux2_state ? "按下/Pressed" : "释放/Released");
}
```

### Python解码示例 / Python Decoding Example
```python
import struct

def decode_remote_data(data_bytes):
    """
    解码遥控器数据 / Decode remote control data
    data_bytes: 10字节的数据包 / 10-byte data packet
    返回: 字典格式的解码数据 / Returns: decoded data in dictionary format
    """
    if len(data_bytes) != 10:
        raise ValueError("数据包必须是10字节 / Data packet must be 10 bytes")
    
    # 解包数据 (小端序) / Unpack data (little endian)
    # 'HHHHBB' = 4个uint16 + 2个uint8
    adc1, adc2, adc3, adc4, aux1, aux2 = struct.unpack('<HHHHBB', data_bytes)
    
    return {
        'adc_channels': {
            'channel_1': adc1,
            'channel_2': adc2, 
            'channel_3': adc3,
            'channel_4': adc4
        },
        'buttons': {
            'aux1': bool(aux1),
            'aux2': bool(aux2)
        },
        'timestamp': time.time()  # 添加时间戳 / Add timestamp
    }

# 使用示例 / Usage Example
def process_nrf24_data(raw_data):
    try:
        decoded = decode_remote_data(raw_data)
        print(f"摇杆数据 / Joystick Data: {decoded['adc_channels']}")
        print(f"按键状态 / Button States: {decoded['buttons']}")
    except Exception as e:
        print(f"解码错误 / Decode Error: {e}")
```

---

## 🔧 接收端NRF24配置 / Receiver NRF24 Configuration

### 必须匹配的配置代码 / Required Matching Configuration Code
```c
// 接收端初始化 / Receiver Initialization
void nrf24_rx_init(void) {
    uint8_t rx_addr[5] = {0x45, 0x55, 0x67, 0x10, 0x21}; // 与发射端相同 / Same as TX
    
    nrf24_init();
    nrf24_listen();                    // 设置为接收模式 / Set to RX mode
    
    // 关键配置 - 必须与发射端一致 / Critical Config - Must match TX
    nrf24_auto_ack_all(auto_ack);      // 自动应答: 启用 / Auto ACK: Enabled
    nrf24_en_ack_pld(disable);         // ACK载荷: 禁用 / ACK Payload: Disabled
    nrf24_dpl(disable);                // 动态载荷: 禁用 / Dynamic Payload: Disabled
    nrf24_set_crc(en_crc, _1byte);     // CRC: 启用1字节 / CRC: 1-byte enabled
    nrf24_data_rate(_1mbps);           // 数据速率: 1Mbps / Data Rate: 1Mbps
    nrf24_set_channel(90);             // 频道: 90 / Channel: 90
    nrf24_set_addr_width(5);           // 地址宽度: 5字节 / Address Width: 5 bytes
    
    // 载荷大小设置 / Payload Size Setup
    nrf24_pipe_pld_size(0, 10);        // 管道0: 10字节 / Pipe 0: 10 bytes
    
    // 自动重传设置 / Auto Retransmission Setup  
    nrf24_auto_retr_delay(4);          // 重传延迟: 1.25ms / Retrans Delay: 1.25ms
    nrf24_auto_retr_limit(10);         // 重传次数: 10次 / Retrans Count: 10
    
    // 打开接收管道 / Open RX Pipe
    nrf24_open_rx_pipe(0, rx_addr);
    
    ce_high(); // 开始监听 / Start listening
}
```

---

## 📋 配置检查清单 / Configuration Checklist

### 发射端和接收端都必须设置 / Both TX and RX Must Set:
- [x] **频道 / Channel**: 90
- [x] **数据速率 / Data Rate**: 1 Mbps  
- [x] **地址 / Address**: {0x45, 0x55, 0x67, 0x10, 0x21}
- [x] **载荷大小 / Payload Size**: 10 bytes
- [x] **CRC**: 启用1字节 / 1-byte enabled
- [x] **自动应答 / Auto ACK**: 启用 / Enabled
- [x] **自动重传 / Auto Retrans**: 10次，1.25ms / 10 times, 1.25ms

### 调试提示 / Debugging Tips
1. **通信测试 / Communication Test**: 先用简单数据测试连接
2. **地址验证 / Address Verification**: 确认TX和RX地址完全相同
3. **功率设置 / Power Settings**: 确保两端都有足够的供电
4. **距离测试 / Range Testing**: 先在近距离测试，再逐步增加距离
5. **干扰检查 / Interference Check**: 避开WiFi等2.4GHz设备

---

## 🚀 快速测试代码 / Quick Test Code

### 接收端测试 / Receiver Test
```c
// 在主循环中 / In main loop
if(nrf24_data_available()) {
    uint8_t rx_data[10];
    nrf24_receive(rx_data, 10);
    
    // 解码并打印 / Decode and print
    RemoteControl_Data_t decoded;
    decode_remote_data(rx_data, &decoded);
    
    printf("收到数据 / Received: ADC=[%d,%d,%d,%d] BTN=[%d,%d]\n",
           decoded.adc_ch1, decoded.adc_ch2, decoded.adc_ch3, 
           decoded.adc_ch4, decoded.aux1_state, decoded.aux2_state);
}
```

---

## 📞 故障排除 / Troubleshooting

| 问题 / Problem | 可能原因 / Possible Cause | 解决方案 / Solution |
|---|---|---|
| 无数据接收 / No Data | 配置不匹配 / Config Mismatch | 检查所有配置参数 / Check all config params |
| 数据丢失 / Data Loss | 距离太远 / Too Far | 减少距离或增加功率 / Reduce distance or increase power |
| 数据错误 / Wrong Data | 字节序问题 / Endian Issue | 检查大小端序 / Check endianness |
| 连接不稳定 / Unstable | 供电不足 / Power Issue | 检查电源和去耦电容 / Check power and decoupling caps |

---

**创建时间 / Created**: $(date)  
**版本 / Version**: 1.0  
**适用于 / Compatible with**: STM32F103 + NRF24L01+ 