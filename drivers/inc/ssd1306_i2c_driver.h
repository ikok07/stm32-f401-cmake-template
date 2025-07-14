//
// Created by Kok on 7/2/25.
//

#ifndef SSD1306_I2C_DRIVER_H
#define SSD1306_I2C_DRIVER_H

#include <i2c_driver.h>
#include <stdint.h>

/*
 * Helpful resource for generating bitmaps from images
 * https://javl.github.io/image2cpp/
 */

/* ------------ COMMUNICATION MACROS ------------ */

#define SSD1306_I2C_ADDR                                     0x3C       // or 0x3D
#define SSD1306_I2C_SEND_BUFFER_LEN                          1025

#define SSD1306_COMM_CONT_BIT_POS                            7
#define SSD1306_COMM_DC_BIT_POS                              6

#define SSD1306_COMM_CMD_BYTE_BIT                            0
#define SSD1306_COMM_DATA_BYTE_BIT                           1

#define SCROLL_DIR_RIGHT                                     0
#define SCROLL_DIR_LEFT                                      1
#define SCROLL_DIR_VRIGHT                                    1
#define SCROLL_DIR_VLEFT                                     2

#define SSD1306_MIN_WIDTH                                    0
#define SSD1306_MAX_WIDTH                                    128
#define SSD1306_MIN_HEIGHT                                   0
#define SSD1306_MAX_HEIGHT                                   64

#define SSD1306_MIN_CONTRAST                                 0
#define SSD1306_MAX_CONTRAST                                 255

#define SSD1306_MIN_V_SCROLL_OFFSET                          0x00
#define SSD1306_MAX_V_SCROLL_OFFSET                          0x3F    // 63

#define SSD1306_MIN_PAGE_LWR_COL_ADDR                        0x00    // Page mode
#define SSD1306_MAX_PAGE_LWR_COL_ADDR                        0xF     // Page mode
#define SSD1306_MIN_PAGE_HIGH_COL_ADDR                       0x10    // Page mode
#define SSD1306_MAX_PAGE_HIGH_COL_ADDR                       0x1F    // Page mode

#define SSD1306_MIN_PAGE_ADDR                                0x00    // Horizontal and Vertical mode
#define SSD1306_MAX_PAGE_ADDR                                0x07    // Horizontal and Vertical mode
#define SSD1306_MIN_COL_ADDR                                 0x00    // Horizontal and Vertical mode
#define SSD1306_MAX_COL_ADDR                                 0x7F    // Horizontal and Vertical mode (127d)

#define SSD1306_DISPLAY_STRT_LINE_MIN                        0x00
#define SSD1306_DISPLAY_STRT_LINE_MAX                        0x3F   // 63
#define SSD1306_MIN_MUX_RATIO                                0xF
#define SSD1306_MAX_MUX_RATIO                                0x3F   // 63
#define SSD1306_MIN_V_OFFSET                                 0x00
#define SSD1306_MAX_V_OFFSET                                 0x3F   // 63

#define SSD1306_MIN_DCLK_DIVIDE_RATIO                        0x00
#define SSD1306_MAX_DCLK_DIVIDE_RATIO                        0xF
#define SSD1306_MIN_OSC_FREQ                                 0x00
#define SSD1306_MAX_OSC_FREQ                                 0xF

#define SSD1306_MIN_PHASE_PRECHARGE_PERIOD                   0x01
#define SSD1306_MAX_PHASE_PRECHARGE_PERIOD                   0xF


/* ------------ COMMANDS MACROS ------------ */

/* ************ Fundamental Commands ************ */

#define SSD1306_CMD_SET_CONTRAST                             0x81
#define SSD1306_CMD_SET_RESUME_DISPLAY_ON                    0xA4
#define SSD1306_CMD_SET_ENTIRE_DISPLAY_ON                    0xA5
#define SSD1306_CMD_SET_NORMAL_DISPLAY                       0xA6
#define SSD1306_CMD_SET_INVERSE_DISPLAY                      0xA7
#define SSD1306_CMD_SET_SLEEP_MODE                           0xAE
#define SSD1306_CMD_SET_NORMAL_MODE                          0xAF

/* ************ Scrolling Commands ************ */

#define SSD1306_CMD_SET_CONT_H_RIGHT_SCROLLING_SETUP         0x26
#define SSD1306_CMD_SET_CONT_H_LEFT_SCROLLING_SETUP          0x27
#define SSD1306_CMD_SET_CONT_VH_RIGHT_SCROLLING_SETUP        0x29
#define SSD1306_CMD_SET_CONT_VH_LEFT_SCROLLING_SETUP         0x2A
#define SSD1306_CMD_SET_SCROLL_DISABLE                       0x2E
#define SSD1306_CMD_SET_SCROLL_ENABLE                        0x2F
#define SSD1306_CMD_SET_V_SCROLL_AREA                        0xA3

/* ************ Address Setting Commands ************ */

#define SSD1306_CMD_SET_MEM_ADDR_MODE                        0x20
#define SSD1306_CMD_SET_COL_ADDR                             0x21
#define SSD1306_CMD_SET_PAGE_ADDR                            0x22

/* ************ Hardware Configuration Commands ************ */

#define SSD1306_CMD_SET_REMAP_START_0                        0xA0
#define SSD1306_CMD_SET_REMAP_START_127                      0xA1
#define SSD1306_CMD_SET_MUX_RATIO                            0xA8
#define SSD1306_CMD_SET_COM_SCAN_DIR_NORMAL                  0xC0
#define SSD1306_CMD_SET_COM_SCAN_DIR_REVERSE                 0xC8
#define SSD1306_CMD_SET_DISPLAY_OFFSET                       0xD3
#define SSD1306_CMD_SET_COM_PINS_HDWR_CONF                   0xDA

/* ************ Timing & Driving Scheme Setting Commands ************ */

#define SSD1306_CMD_SET_DCLK_DIV_RATIO_OSC_FREQ              0xD5
#define SSD1306_CMD_SET_PRE_CHARGE_PERIOD                    0xD9
#define SSD1306_CMD_SET_VCOMH_DES_LVL                        0xDB
#define SSD1306_CMD_NOOP                                     0xE3

/* ************ Charge Pump Commands ************ */

#define SSD1306_CMD_SET_CHARGE_PUMP                         0x8D

/* ------------ ERROR CODES ------------ */
typedef enum {
    SSD1306_ErrOK,
    SSD1306_ErrComm,
    SSD1306_ErrI2CNotEnabled,
    SSD1306_ErrI2CIncorrectSpeed,
    SSD1306_ErrNotInitialized,
    SSD1306_ErrInvalidArg,
    SSD1306_ErrBufferOverflow,
    SSD1306_ErrInvalidAddrMode,
    SSD1306_ErrOutOfBounds,
    SSD1306_ErrInvalidFont
} SSD1306_Error_e;

typedef enum {
    SSD1306_DisplayModeSleep,
    SSD1306_DisplayModeNormal,
} SSD1306_DisplayMode;

typedef enum {
    SSD1306_HScrollRight,               // Should be used only in horizontal scroll mode
    SSD1306_HScrollLeft,                // Should be used only in horizontal scroll mode
    SSD1306_VHScrollLeft,               // Should be used only in vertical scroll mode
    SSD1306_VHScrollRight,              // Should be used only in vertical scroll mode
} SSD1306_ScrollDirection_e;

typedef enum {
    SSD1306_Page0,
    SSD1306_Page1,
    SSD1306_Page2,
    SSD1306_Page3,
    SSD1306_Page4,
    SSD1306_Page5,
    SSD1306_Page6,
    SSD1306_Page7,
} SSD1306_Page_e;

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
    SSD1306_MemAddrPage = 2,
} SSD1306_MemoryAddressingMode_e;

typedef enum {
    SSD1306_VCOMHLevelMode0,        // ∼ 0.65 * Vcc
    SSD1306_VCOMHLevelMode1 = 2,    // ∼ 0.77 * Vcc
    SSD1306_VCOMHLevelMode2,        // ∼ 0.83 * Vcc
} SSD1306_VCOMHDeselectLevel_e;

typedef enum {
    SSD1306_Font8x8,
    SSD1306_Font8x16,
    SSD1306_Font16x15,
} SSD1306_Font_e;

/* ------------ CONFIG STRUCTURES ------------ */

typedef struct {
    uint8_t Enabled;
    SSD1306_ScrollDirection_e Direction;
    SSD1306_Page_e StartPageAddress;
    SSD1306_Page_e EndPageAddress;
    SSD1306_ScrollStepInterval_e ScrollStepInterval;
    uint8_t StartRow;
    uint8_t NumberOfRows;
    uint8_t VerticalOffset;
} SSD1306_ScrollState_t;

typedef struct {
    SSD1306_MemoryAddressingMode_e AddressingMode;
    uint8_t ColLowerNibble;         // Page mode
    uint8_t ColHigherNibble;        // Page mode
    uint8_t ColStartAddr;           // Horizontal and Vertical mode
    uint8_t ColEndAddr;             // Horizontal and Vertical mode
    SSD1306_Page_e PageStartAddr;
    SSD1306_Page_e PageEndAddr;
} SSD1306_AddressingState_t;

typedef struct {
    uint8_t StartLine;
    uint8_t SegmentRemapped;                // ENABLED => col address 127 is mapped to SEG0; DISABLED => address 0 is mapped to SEG0
    uint8_t MuxRatio;                       // Min - SSD1306_MIN_MUX_RATIO; Max - SSD1306_MAX_MUX_RATIO
    uint8_t COMScanRemapped;                // ENABLED => Scan from COM[N-1] to COM0; DISABLED => Scan from COM0 to COM[N-1]
    uint8_t DisplayVOffset;                 // Min - SSD1306_MIN_V_OFFSET; Max - SSD1306_MAX_V_OFFSET
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
    uint8_t PowerEnabled;
    uint8_t Contrast;
    uint8_t DisplayInversed;
    uint8_t EntireDisplayON;
    uint8_t ChargePumpEnabled;
    SSD1306_DisplayMode DisplayMode;
    SSD1306_ScrollState_t ScrollState;
    SSD1306_AddressingState_t AddressingState;
    SSD1306_HardwareState_t HardwareState;
    SSD1306_TimingState_t TimingState;
} SSD1306_State_t;

typedef struct {
    uint8_t MUXRatio;
    uint8_t DisplayOffset;
    uint8_t DisplayStartLine;
    uint8_t SegmentsRemapped;
    uint8_t COMScanRemapped;
    uint8_t AlternativeCOMPinConfigEnabled;
    uint8_t COMLeftRightRemapEnabled;
    uint8_t Contrast;
    uint8_t DivideRatio;
    uint8_t OSCFreq;
    SSD1306_Font_e Font;
    uint8_t FontMirrored;
} SSD1306_Config_t;

/**
 * @brief The display's VCC should be connected to a GPIO in order to be physically reset when needed
 */
typedef struct {
    GPIO_TypeDef *pGPIOx;
    uint8_t PinNumber;
} SSD1306_PwrGPIOConfig;

typedef struct {
    SSD1306_Font_e Font;
    uint8_t Mirrored;
    uint8_t Width;
    uint8_t Height;
} SSD1306_FontConfig_t;

typedef struct {
    I2C_Handle_t *I2CHandle;
    SSD1306_PwrGPIOConfig PowerConfig;
    SSD1306_FontConfig_t FontConfig;
    SSD1306_State_t DeviceState;
} SSD1306_Handle_t;

/* ------------ DATA DISPLAY METHODS ------------ */

// Fonts
SSD1306_Error_e SSD1306_SetFont(SSD1306_Handle_t *pSSD1306Handle, SSD1306_Font_e Font, uint8_t Mirrored);

// Page addressing mode
SSD1306_Error_e SSD1306_SetCursor(SSD1306_Handle_t *pSSD1306Handle, uint8_t Column, SSD1306_Page_e Page);
SSD1306_Error_e SSD1306_Write(SSD1306_Handle_t *pSSD1306Handle, char *str);
SSD1306_Error_e SSD1306_Clear(SSD1306_Handle_t *pSSD1306Handle);

// Horizontal / Vertical addressing mode
SSD1306_Error_e SSD1306_SetWriteAreaH(SSD1306_Handle_t *pSSD1306Handle, uint8_t xStart, uint8_t xEnd, uint8_t yStart, uint8_t yEnd);
SSD1306_Error_e SSD1306_WriteH(SSD1306_Handle_t *pSSD1306Handle, uint8_t x, uint8_t y, char *str);
SSD1306_Error_e SSD1306_DrawH(SSD1306_Handle_t *pSSD1306Handle, uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t *bitmap, uint8_t len);
SSD1306_Error_e SSD1306_ClearH(SSD1306_Handle_t *pSSD1306Handle);
SSD1306_Error_e SSD1306_UpdateH(SSD1306_Handle_t *pSSD1306Handle);


/* ------------ COMMAND METHODS ------------ */

/*
 * Init
 */
SSD1306_Error_e SSD1306_Init(SSD1306_Handle_t *pSSD1306Handle, SSD1306_Config_t Config);
void SSD1306_DeInit(SSD1306_Handle_t *pSSD1306Handle);

/*
 * Fundamentals
 */
SSD1306_Error_e SSD1306_SetContrast(SSD1306_Handle_t *pSSD1306Handle, uint8_t Value);
SSD1306_Error_e SSD1306_ResumeDisplay(SSD1306_Handle_t *pSSD1306Handle);
SSD1306_Error_e SSD1306_SetEntireDisplayON(SSD1306_Handle_t *pSSD1306Handle);               // Used for debugging
SSD1306_Error_e SSD1306_DisplayControl(SSD1306_Handle_t *pSSD1306Handle, uint8_t Enabled);

/*
 * Scrolling
 */
SSD1306_Error_e SSD1306_SetupHorizontalScroll(SSD1306_Handle_t *pSSD1306Handle, SSD1306_ScrollDirection_e Direction, SSD1306_Page_e StartPage, SSD1306_Page_e EndPage, SSD1306_ScrollStepInterval_e StepInterval);
SSD1306_Error_e SSD1306_SetupVerticalHorizontalScroll(SSD1306_Handle_t *pSSD1306Handle, SSD1306_ScrollDirection_e Direction, SSD1306_Page_e StartPage, SSD1306_Page_e EndPage, SSD1306_ScrollStepInterval_e StepInterval, uint8_t VerticalOffset);
SSD1306_Error_e SSD1306_DeactivateScroll(SSD1306_Handle_t *pSSD1306Handle);
SSD1306_Error_e SSD1306_ActivateScroll(SSD1306_Handle_t *pSSD1306Handle);
SSD1306_Error_e SSD1306_SetVerticalScrollArea(SSD1306_Handle_t *pSSD1306Handle, uint8_t StartRow, uint8_t NumberOfRows);

/*
 * Addressing
 */
SSD1306_Error_e SSD1306_SetPageModeColumnLowerNibble(SSD1306_Handle_t *pSSD1306Handle, uint8_t ColLowerAddr);
SSD1306_Error_e SSD1306_SetPageModeColumnHigherNibble(SSD1306_Handle_t *pSSD1306Handle, uint8_t ColHigherAddr);
SSD1306_Error_e SSD1306_SetPageModeStartPage(SSD1306_Handle_t *pSSD1306Handle, SSD1306_Page_e Page);
SSD1306_Error_e SSD1306_SetMemoryAddrMode(SSD1306_Handle_t *pSSD1306Handle, SSD1306_MemoryAddressingMode_e Mode);
SSD1306_Error_e SSD1306_SetHModeColumnAddr(SSD1306_Handle_t *pSSD1306Handle, uint8_t StartAddr, uint8_t EndAddr);
SSD1306_Error_e SSD1306_SetHModePageAddr(SSD1306_Handle_t *pSSD1306Handle, SSD1306_Page_e StartPage, SSD1306_Page_e EndPage);

/*
 * Hardware Configuration
 */
SSD1306_Error_e SSD1306_SetStartLine(SSD1306_Handle_t *pSSD1306Handle, uint8_t StartLine);
SSD1306_Error_e SSD1306_SetSegmentRemap(SSD1306_Handle_t *pSSD1306Handle, uint8_t Enabled);
SSD1306_Error_e SSD1306_SetMuxRatio(SSD1306_Handle_t *pSSD1306Handle, uint8_t Ratio);
SSD1306_Error_e SSD1306_SetCOMScanRemapping(SSD1306_Handle_t *pSSD1306Handle, uint8_t Enabled);
SSD1306_Error_e SSD1306_SetDisplayOffset(SSD1306_Handle_t *pSSD1306Handle, uint8_t Value);
SSD1306_Error_e SSD1306_ConfigureCOMPins(SSD1306_Handle_t *pSSD1306Handle, uint8_t AlternativeCOMPinConfigEnabled, uint8_t COMLeftRightRemapEnabled);

/*
 * Timing & Driving Scheme
 */
SSD1306_Error_e SSD1306_SetDivideRatioAndOSCFreq(SSD1306_Handle_t *pSSD1306Handle, uint8_t DivideRatio, uint8_t OSCFreq);
SSD1306_Error_e SSD1306_SetPreChargePeriod(SSD1306_Handle_t *pSSD1306Handle, uint8_t PhaseOnePeriod, uint8_t PhaseTwoPeriod);
SSD1306_Error_e SSD1306_SetVCOMHDeselectLevel(SSD1306_Handle_t *pSSD1306Handle, SSD1306_VCOMHDeselectLevel_e Level);

/*
 * Charge Pump
 */
SSD1306_Error_e SSD1306_ChargePumpControl(SSD1306_Handle_t *pSSD1306Handle, uint8_t Enabled);

/**
 * Power
 */
void SSD1306_PowerControl(SSD1306_Handle_t *pSSD1306Handle, uint8_t Enabled);

#endif //SSD1306_I2C_DRIVER_H
