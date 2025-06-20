# NRF24 �������ú����ݽ����ĵ�
# NRF24 Configuration Setup and Data Decoding Documentation

## �9�9 ��Ҫ���� / Important Notice
**����NRF24ģ�飨����˺ͽ��նˣ�����ʹ����ȫ��ͬ�����ò������ܽ���ͨ�ţ�**
**Both NRF24 modules (TX and RX) MUST use identical configuration parameters to establish communication!**

---

## �9�9 ��ǰ��������� / Current Transmitter Configuration

### Ӳ������ / Hardware Connections
```
STM32F103 �� NRF24L01+
------------------------
SPI2_SCK  (PB13) �� SCK
SPI2_MISO (PB14) �� MISO  
SPI2_MOSI (PB15) �� MOSI
PB12             �� CSN (Chip Select)
PA11             �� CE  (Chip Enable)
PA8              �� IRQ (Interrupt)
3.3V             �� VCC
GND              �� GND
```

### ͨ�Ų��� / Communication Parameters
```c
// ����ƥ��Ĳ��� / Parameters that MUST match:
Ƶ�� / Channel:           90 (2.490 GHz)
�������� / Data Rate:     1 Mbps
���书�� / TX Power:      0 dBm (����� / Maximum)
��ַ��� / Address Width: 5 bytes
�����ַ / TX Address:    {0x45, 0x55, 0x67, 0x10, 0x21}
�غɴ�С / Payload Size:  10 bytes (�̶� / Fixed)
CRCУ�� / CRC:           ���ã�1�ֽ� / Enabled, 1-byte
�Զ�Ӧ�� / Auto ACK:      ���� / Enabled
�Զ��ش� / Auto Retrans:  10�����ԣ�1.25ms�ӳ� / 10 retries, 1.25ms delay
```

---

## �9�6 TX Buffer ���ݸ�ʽ / TX Buffer Data Format

### ���ݽṹ / Data Structure (10 bytes total)
```
Byte λ�� | ��������        | ��������    | ����
Byte Pos | Data Content   | Data Type  | Description
---------|---------------|------------|------------------
0-1      | ADC Channel 1 | uint16_t   | ҡ��X�� / Joystick X-axis
2-3      | ADC Channel 2 | uint16_t   | ҡ��Y�� / Joystick Y-axis  
4-5      | ADC Channel 3 | uint16_t   | ҡ��Z�� / Joystick Z-axis
6-7      | ADC Channel 4 | uint16_t   | ����/���� / Throttle/Other
8        | AUX1 State    | uint8_t    | ��������1 / Auxiliary Button 1
9        | AUX2 State    | uint8_t    | ��������2 / Auxiliary Button 2
```

### ���ݷ�Χ / Data Ranges
```c
ADC ͨ�� / ADC Channels (0-7�ֽ� / Bytes 0-7):
- ��������: 16λ�޷������� / Data Type: 16-bit unsigned integer
- ��ֵ��Χ: 0 - 4095 (12λADC / 12-bit ADC)
- �ֽ���: С���� / Byte Order: Little Endian
- ����ֵ: 2048 (Լ1.65V / ~1.65V)

����״̬ / Button States (8-9�ֽ� / Bytes 8-9):
- ��������: 8λ�޷������� / Data Type: 8-bit unsigned integer  
- ��ֵ: 0 = δ���� / Not Pressed, 1 = ���� / Pressed
```

---

## �9�9 ���ն����ݽ��� / Receiver Data Decoding

### C���Խ���ʾ�� / C Language Decoding Example
```c
#include <stdint.h>

// ���ݽṹ���� / Data Structure Definition
typedef struct {
    uint16_t adc_ch1;      // ADCͨ��1 / ADC Channel 1
    uint16_t adc_ch2;      // ADCͨ��2 / ADC Channel 2  
    uint16_t adc_ch3;      // ADCͨ��3 / ADC Channel 3
    uint16_t adc_ch4;      // ADCͨ��4 / ADC Channel 4
    uint8_t aux1_state;    // AUX1����״̬ / AUX1 Button State
    uint8_t aux2_state;    // AUX2����״̬ / AUX2 Button State
} __attribute__((packed)) RemoteControl_Data_t;

// ���뺯�� / Decoding Function
void decode_remote_data(uint8_t* rx_buffer, RemoteControl_Data_t* data) {
    // ����1: ֱ���ڴ濽�� / Method 1: Direct memory copy
    memcpy(data, rx_buffer, sizeof(RemoteControl_Data_t));
    
    // ����2: ���ֽڽ��� / Method 2: Byte-by-byte parsing
    data->adc_ch1 = (rx_buffer[1] << 8) | rx_buffer[0];
    data->adc_ch2 = (rx_buffer[3] << 8) | rx_buffer[2];
    data->adc_ch3 = (rx_buffer[5] << 8) | rx_buffer[4];
    data->adc_ch4 = (rx_buffer[7] << 8) | rx_buffer[6];
    data->aux1_state = rx_buffer[8];
    data->aux2_state = rx_buffer[9];
}

// ʹ��ʾ�� / Usage Example
void process_received_data(uint8_t* rx_buffer) {
    RemoteControl_Data_t remote_data;
    decode_remote_data(rx_buffer, &remote_data);
    
    // ��ӡ���������� / Print decoded data
    printf("ADC1: %d, ADC2: %d, ADC3: %d, ADC4: %d\n", 
           remote_data.adc_ch1, remote_data.adc_ch2, 
           remote_data.adc_ch3, remote_data.adc_ch4);
    printf("AUX1: %s, AUX2: %s\n", 
           remote_data.aux1_state ? "����/Pressed" : "�ͷ�/Released",
           remote_data.aux2_state ? "����/Pressed" : "�ͷ�/Released");
}
```

### Python����ʾ�� / Python Decoding Example
```python
import struct

def decode_remote_data(data_bytes):
    """
    ����ң�������� / Decode remote control data
    data_bytes: 10�ֽڵ����ݰ� / 10-byte data packet
    ����: �ֵ��ʽ�Ľ������� / Returns: decoded data in dictionary format
    """
    if len(data_bytes) != 10:
        raise ValueError("���ݰ�������10�ֽ� / Data packet must be 10 bytes")
    
    # ������� (С����) / Unpack data (little endian)
    # 'HHHHBB' = 4��uint16 + 2��uint8
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
        'timestamp': time.time()  # ���ʱ��� / Add timestamp
    }

# ʹ��ʾ�� / Usage Example
def process_nrf24_data(raw_data):
    try:
        decoded = decode_remote_data(raw_data)
        print(f"ҡ������ / Joystick Data: {decoded['adc_channels']}")
        print(f"����״̬ / Button States: {decoded['buttons']}")
    except Exception as e:
        print(f"������� / Decode Error: {e}")
```

---

## �9�9 ���ն�NRF24���� / Receiver NRF24 Configuration

### ����ƥ������ô��� / Required Matching Configuration Code
```c
// ���ն˳�ʼ�� / Receiver Initialization
void nrf24_rx_init(void) {
    uint8_t rx_addr[5] = {0x45, 0x55, 0x67, 0x10, 0x21}; // �뷢�����ͬ / Same as TX
    
    nrf24_init();
    nrf24_listen();                    // ����Ϊ����ģʽ / Set to RX mode
    
    // �ؼ����� - �����뷢���һ�� / Critical Config - Must match TX
    nrf24_auto_ack_all(auto_ack);      // �Զ�Ӧ��: ���� / Auto ACK: Enabled
    nrf24_en_ack_pld(disable);         // ACK�غ�: ���� / ACK Payload: Disabled
    nrf24_dpl(disable);                // ��̬�غ�: ���� / Dynamic Payload: Disabled
    nrf24_set_crc(en_crc, _1byte);     // CRC: ����1�ֽ� / CRC: 1-byte enabled
    nrf24_data_rate(_1mbps);           // ��������: 1Mbps / Data Rate: 1Mbps
    nrf24_set_channel(90);             // Ƶ��: 90 / Channel: 90
    nrf24_set_addr_width(5);           // ��ַ���: 5�ֽ� / Address Width: 5 bytes
    
    // �غɴ�С���� / Payload Size Setup
    nrf24_pipe_pld_size(0, 10);        // �ܵ�0: 10�ֽ� / Pipe 0: 10 bytes
    
    // �Զ��ش����� / Auto Retransmission Setup  
    nrf24_auto_retr_delay(4);          // �ش��ӳ�: 1.25ms / Retrans Delay: 1.25ms
    nrf24_auto_retr_limit(10);         // �ش�����: 10�� / Retrans Count: 10
    
    // �򿪽��չܵ� / Open RX Pipe
    nrf24_open_rx_pipe(0, rx_addr);
    
    ce_high(); // ��ʼ���� / Start listening
}
```

---

## �9�7 ���ü���嵥 / Configuration Checklist

### ����˺ͽ��ն˶��������� / Both TX and RX Must Set:
- [x] **Ƶ�� / Channel**: 90
- [x] **�������� / Data Rate**: 1 Mbps  
- [x] **��ַ / Address**: {0x45, 0x55, 0x67, 0x10, 0x21}
- [x] **�غɴ�С / Payload Size**: 10 bytes
- [x] **CRC**: ����1�ֽ� / 1-byte enabled
- [x] **�Զ�Ӧ�� / Auto ACK**: ���� / Enabled
- [x] **�Զ��ش� / Auto Retrans**: 10�Σ�1.25ms / 10 times, 1.25ms

### ������ʾ / Debugging Tips
1. **ͨ�Ų��� / Communication Test**: ���ü����ݲ�������
2. **��ַ��֤ / Address Verification**: ȷ��TX��RX��ַ��ȫ��ͬ
3. **�������� / Power Settings**: ȷ�����˶����㹻�Ĺ���
4. **������� / Range Testing**: ���ڽ�������ԣ��������Ӿ���
5. **���ż�� / Interference Check**: �ܿ�WiFi��2.4GHz�豸

---

## �0�4 ���ٲ��Դ��� / Quick Test Code

### ���ն˲��� / Receiver Test
```c
// ����ѭ���� / In main loop
if(nrf24_data_available()) {
    uint8_t rx_data[10];
    nrf24_receive(rx_data, 10);
    
    // ���벢��ӡ / Decode and print
    RemoteControl_Data_t decoded;
    decode_remote_data(rx_data, &decoded);
    
    printf("�յ����� / Received: ADC=[%d,%d,%d,%d] BTN=[%d,%d]\n",
           decoded.adc_ch1, decoded.adc_ch2, decoded.adc_ch3, 
           decoded.adc_ch4, decoded.aux1_state, decoded.aux2_state);
}
```

---

## �9�6 �����ų� / Troubleshooting

| ���� / Problem | ����ԭ�� / Possible Cause | ������� / Solution |
|---|---|---|
| �����ݽ��� / No Data | ���ò�ƥ�� / Config Mismatch | ����������ò��� / Check all config params |
| ���ݶ�ʧ / Data Loss | ����̫Զ / Too Far | ���پ�������ӹ��� / Reduce distance or increase power |
| ���ݴ��� / Wrong Data | �ֽ������� / Endian Issue | ����С���� / Check endianness |
| ���Ӳ��ȶ� / Unstable | ���粻�� / Power Issue | ����Դ��ȥ����� / Check power and decoupling caps |

---

**����ʱ�� / Created**: $(date)  
**�汾 / Version**: 1.0  
**������ / Compatible with**: STM32F103 + NRF24L01+ 