//
// Created by Kok on 7/2/25.
//

#ifndef SSD1306_I2C_DRIVER_H
#define SSD1306_I2C_DRIVER_H

#include <stdint.h>

/* ------------ COMMUNICATION MACROS ------------ */

#define DS1306_COMM_CONTINUATION_DISABLED                   0          // Indicates this is the last command byte in the current transmission
#define DS1306_COMM_CONTINUATION_ENABLED                    1          // Indicates that another command byte follows

#define DS1306_COMM_CMD_BYTE_BIT                            0
#define DS1306_COMM_DATA_BYTE_BIT                           1

#define SCROLL_DIR_RIGHT                                    0
#define SCROLL_DIR_LEFT                                     1
#define SCROLL_DIR_VRIGHT                                   1
#define SCROLL_DIR_VLEFT                                    2

#define DS1306_MIN_PAGE_LWR_COL_ADDR                        0x00
#define DS1306_MAX_PAGE_LWR_COL_ADDR                        0xF
#define DS1306_MIN_PAGE_HIGH_COL_ADDR                       0x10
#define DS1306_MAX_PAGE_HIGH_COL_ADDR                       0x1F
#define DS1306_MIN_PAGE_STRT_ADDR                           0xB0
#define DS1306_MAX_PAGE_STRT_ADDR                           0xB7

#define DS1306_DISPLAY_STRT_LINE_MIN                        0x40
#define DS1306_DISPLAY_STRT_LINE_MAX                        0x7F
#define DS1306_MIN_MUX_RATIO                                0xF;
#define DS1306_MAX_MUX_RATIO                                0x40;
#define DS1306_MIN_V_OFFSET                                 0x00;
#define DS1306_MAX_V_OFFSET                                 0x3F;   // 63


/* ------------ COMMANDS MACROS ------------ */

/* ************ Fundamental Commands ************ */

#define DS1306_CMD_SET_CONTRAST_CONTROL                     0x81
#define DS1306_CMD_SET_RESUME_DISPLAY_ON                    0xA4
#define DS1306_CMD_SET_ENTIRE_DISPLAY_ON                    0xA5
#define DS1306_CMD_SET_NORMAL_DISPLAY                       0xA6
#define DS1306_CMD_SET_INVERSE_DISPLAY                      0xA7
#define DS1306_CMD_SET_SLEEP_MODE                           0xAE
#define DS1306_CMD_SET_NORMAL_MODE                          0xAF

/* ************ Scrolling Commands ************ */

#define DS1306_CMD_SET_CONT_H_RIGHT_SCROLLING_SETUP         0x26
#define DS1306_CMD_SET_CONT_H_LEFT_SCROLLING_SETUP          0x27
#define DS1306_CMD_SET_CONT_VH_RIGHT_SCROLLING_SETUP        0x29
#define DS1306_CMD_SET_CONT_VH_LEFT_SCROLLING_SETUP         0x2A
#define DS1306_CMD_SET_SCROLL_DISABLE                       0x2E
#define DS1306_CMD_SET_SCROLL_ENABLE                        0x2F
#define DS1306_CMD_SET_V_SCROLL_AREA                        0xA3

/* ************ Address Setting Commands ************ */

#define DS1306_CMD_SET_MEM_ADDR_MODE                        0x20
#define DS1306_CMD_SET_COL_ADDR                             0x21
#define DS1306_CMD_SET_PAGE_ADDR                            0x22

/* ************ Hardware Configuration Commands ************ */

#define DS1306_CMD_SET_REMAP_START_0                        0xA0
#define DS1306_CMD_SET_REMAP_START_1                        0xA1
#define DS1306_CMD_SET_MUX_RATIO                            0xA8
#define DS1306_CMD_SET_COM_SCAN_DIR_NORMAL                  0xC0
#define DS1306_CMD_SET_COM_SCAN_DIR_REVERSE                 0xC8
#define DS1306_CMD_SET_DISPLAY_OFFSET                       0xD3
#define DS1306_CMD_SET_COM_PINS_HDWR_CONF                   0xDA

/* ************ Timing & Driving Scheme Setting Commands ************ */

#define DS1306_CMD_SET_DCLK_DIV_RATIO_OSC_FREQ              0xD5
#define DS1306_CMD_SET_PRE_CHARGE_PERIOD                    0xD9
#define DS1306_CMD_SET_VCOMH_DES_LVL                        0xDB
#define DS1306_CMD_NOOP                                     0xE3

/* ------------ ERROR CODES ------------ */
typedef enum {
    SSD1306_ErrOK
} SSD1306_Error_e;

typedef enum {
    SSD1306_DisplayModeSleep,
    SSD1306_DisplayModeNormal,
} SSD1306_DisplayMode;

typedef enum {
    SSD1306_HScrollRight,
    SSD1306_HScrollLeft,
    SSD1306_VHScrollLeft,
    SSD1306_VHScrollRight,
} SSD1306_ScrollDirection_e;

typedef enum {
    SSD1306_ScrollInterval5Frames,
    SSD1306_ScrollInterval64Frames,
    SSD1306_ScrollInterval128Frames,
    SSD1306_ScrollInterval256Frames,
    SSD1306_ScrollInterval3Frames,
    SSD1306_ScrollInterval4Frames,
    SSD1306_ScrollInterval25Frames,
    SSD1306_ScrollInterval2Frames,
} SSD1306_ScrollStepInterval_e;

typedef enum {
    SSD1306_MemAddrHorizontal,
    SSD1306_MemAddrVertical,
    SSD1306_MemAddrPage,
} SSD1306_MemoryAddressingMode_e;

typedef enum {
    SSD1306_VCOMHLevelMode0,        // ∼ 0.65 * Vcc
    SSD1306_VCOMHLevelMode1,        // ∼ 0.77 * Vcc
    SSD1306_VCOMHLevelMode2,        // ∼ 0.83 * Vcc
} SSD1306_VCOMHDeselectLevel_e;

typedef struct {
    uint8_t Enabled;
    SSD1306_ScrollDirection_e Direction;
    uint8_t StartPageAddress;
    uint8_t EndPageAddress;
    SSD1306_ScrollStepInterval_e ScrollStepInterval;
    uint8_t StartRow;
    uint8_t NumberOfRows;
} SSD1306_ScrollState_t;

typedef struct {
    SSD1306_MemoryAddressingMode_e AddressingMode;
    uint8_t ColStartAddr;
    uint8_t ColEndAddr;
    uint8_t PageStartAddr;
    uint8_t PageEndAddr;
} SSD1306_AddressingState_t;

typedef struct {
    uint8_t StartLine;
    uint8_t SegmentRemapped;                // ENABLED => col address 127 is mapped to SEG0; DISABLED => address 0 is mapped to SEG0
    uint8_t MuxRatio;                       // Min - DS1306_MIN_MUX_RATIO; Max - DS1306_MAX_MUX_RATIO
    uint8_t COMScanRemapped;                // ENABLED => Scan from COM[N-1] to COM0; DISABLED => Scan from COM0 to COM[N-1]
    uint8_t DisplayVOffset;                 // Min - DS1306_MIN_V_OFFSET; Max - DS1306_MAX_V_OFFSET
    uint8_t AlternativeCOMPinConfig;
    uint8_t COMLeftRightRemapEnabled;
} SSD1306_HardwareState_t;

typedef struct {
    uint8_t DCLKDivRatio;                   // Display clock divide ratio
    uint8_t OSCFreq;                        // Oscillator frequency
    uint8_t Phase1Period;
    uint8_t Phase2Period;
    SSD1306_VCOMHDeselectLevel_e VCOMHDeselectLevel;
} SSD1306_TimingState_t;

typedef struct {
    uint8_t Contrast;
    uint8_t DisplayEnabled;
    uint8_t DisplayInversed;
    SSD1306_DisplayMode DisplayMode;
    SSD1306_ScrollState_t ScrollState;
    SSD1306_AddressingState_t AddressingState;
    SSD1306_HardwareState_t HardwareState;
    SSD1306_TimingState_t TimingState;
} SSD1306_State_t;

typedef struct {
    SSD1306_State_t DeviceState;
} SSD1306_Handle_t;

/* ------------ CONFIG STRUCTURES ------------ */


#endif //SSD1306_I2C_DRIVER_H
