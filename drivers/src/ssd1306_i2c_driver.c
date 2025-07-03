//
// Created by Kok on 7/2/25.
//

#include "ssd1306_i2c_driver.h"

#include <string.h>
#include "commons.h"

static uint8_t buffer[SSD1306_I2C_BUFFER_MAX_LEN];

static SSD1306_Error_e send_commands(SSD1306_Handle_t *pSSD1306Handle, uint8_t *commands, uint8_t len);
static SSD1306_Error_e send_data(SSD1306_Handle_t *pSSD1306Handle, uint8_t *data, uint8_t len);
static void reset_state(SSD1306_Handle_t *pSSD1306Handle);

#define SSD1306_SEND_COMMANDS(handle, err, ...)         do {                                                        \
                                                            uint8_t cmds[] = {__VA_ARGS__};                         \
                                                            err = send_commands(handle, cmds, sizeof(cmds));        \
                                                        } while (0)

/**
 * @brief Initializes the device I2C communication
 * @note Before calling this method the required GPIOS MUST be configured
 * @param pSSD1306Handle Device handle
 */
SSD1306_Error_e SSD1306_Init(SSD1306_Handle_t *pSSD1306Handle) {
    pSSD1306Handle->Config.I2CHandle->I2C_Config.I2C_DeviceAddress = I2C_DeviceAddr7Bits;
    pSSD1306Handle->Config.I2CHandle->I2C_Config.I2C_SCLSpeed = I2C_SclSpeedSM;
    pSSD1306Handle->Config.I2CHandle->I2C_Config.I2C_FMDutyCycle = I2C_FmDuty2;

    I2C_Error_e err;

    I2C_PeriClockControl(pSSD1306Handle->Config.I2CHandle->pI2Cx, ENABLE);

    if ((err = I2C_Init(pSSD1306Handle->Config.I2CHandle)) != I2C_ErrOK) {
        return SSD1306_ErrComm;
    }

    reset_state(pSSD1306Handle);
    pSSD1306Handle->DeviceInitialized = ENABLE;
    return SSD1306_ErrOK;
}

/**
 * @brief De-initializes the display and the it's corresponding I2C peripheral
 * @param pSSD1306Handle Device handle
 */
void SSD1306_DeInit(SSD1306_Handle_t *pSSD1306Handle) {
    I2C_DeInit(pSSD1306Handle->Config.I2CHandle->pI2Cx);
    reset_state(pSSD1306Handle);
    pSSD1306Handle->DeviceInitialized = DISABLE;
}

/**
 * @brief Sets the desired display contrast
 * @param pSSD1306Handle Device handle
 * @param Value Value between 0 and 255
 */
SSD1306_Error_e SSD1306_SetContrast(SSD1306_Handle_t *pSSD1306Handle, uint8_t Value) {
    if (Value < SSD1306_MIN_CONTRAST || Value > SSD1306_MAX_CONTRAST) return SSD1306_ErrInvalidArg;

    SSD1306_Error_e err = SSD1306_ErrOK;

    SSD1306_SEND_COMMANDS(pSSD1306Handle, err, SSD1306_CMD_SET_CONTRAST, Value);
    if (err != SSD1306_ErrOK) return err;

    pSSD1306Handle->DeviceState.Contrast = Value;
    return err;
}

/**
 * @brief Resumes the display with the data inside the device's RAM
 * @param pSSD1306Handle Device handle
 */
SSD1306_Error_e SSD1306_ResumeDisplay(SSD1306_Handle_t *pSSD1306Handle) {
    SSD1306_Error_e err = SSD1306_ErrOK;

    SSD1306_SEND_COMMANDS(pSSD1306Handle, err, SSD1306_CMD_SET_RESUME_DISPLAY_ON);
    if (err != SSD1306_ErrOK) return err;

    pSSD1306Handle->DeviceState.EntireDisplayON = DISABLE;
    return err;
}

/**
 * @brief Enables all display pixels in order to debug the device
 * @param pSSD1306Handle Device handle
 */
SSD1306_Error_e SSD1306_SetEntireDisplayON(SSD1306_Handle_t *pSSD1306Handle) {
    SSD1306_Error_e err = SSD1306_ErrOK;

    SSD1306_SEND_COMMANDS(pSSD1306Handle, err, SSD1306_CMD_SET_ENTIRE_DISPLAY_ON);
    if (err != SSD1306_ErrOK) return err;

    pSSD1306Handle->DeviceState.EntireDisplayON = ENABLE;
    return err;
}

/**
 * @brief Controls whether the display should be in normal or in sleep mode
 * @param pSSD1306Handle Device handle
 * @param Enabled If the display should be enabled or in sleep mode
 */
SSD1306_Error_e SSD1306_DisplayControl(SSD1306_Handle_t *pSSD1306Handle, uint8_t Enabled) {
    SSD1306_Error_e err = SSD1306_ErrOK;

    if (Enabled) {
        SSD1306_SEND_COMMANDS(pSSD1306Handle, err, SSD1306_CMD_SET_NORMAL_MODE);
        pSSD1306Handle->DeviceState.DisplayMode = SSD1306_DisplayModeNormal;
    } else {
        SSD1306_SEND_COMMANDS(pSSD1306Handle, err, SSD1306_CMD_SET_SLEEP_MODE);
        pSSD1306Handle->DeviceState.DisplayMode = SSD1306_DisplayModeSleep;
    }

    if (err != SSD1306_ErrOK) return err;
    return err;
}

/**
 * @brief Setups the display for horizontal scrolling
 * @param pSSD1306Handle Device handle
 * @param Direction The desired scroll direction (only left / right)
 * @param StartPage The desired start page
 * @param EndPage The desired end page
 * @param StepInterval The time interval between each scroll step in terms of frame frequency
 */
SSD1306_Error_e SSD1306_SetupHorizontalScroll(SSD1306_Handle_t *pSSD1306Handle, SSD1306_ScrollDirection_e Direction,
                                              SSD1306_Page_e StartPage, SSD1306_Page_e EndPage, SSD1306_ScrollStepInterval_e StepInterval) {
    if (Direction >= SSD1306_VHScrollLeft || EndPage < StartPage) return SSD1306_ErrInvalidArg;
    SSD1306_Error_e err = SSD1306_ErrOK;

    uint8_t dirCommand = Direction == SSD1306_HScrollLeft ? SSD1306_CMD_SET_CONT_H_LEFT_SCROLLING_SETUP : SSD1306_CMD_SET_CONT_H_RIGHT_SCROLLING_SETUP;

    SSD1306_SEND_COMMANDS(pSSD1306Handle, err, dirCommand, 0x00, StartPage, StepInterval, EndPage, 0x00, 0xFF);
    if (err != SSD1306_ErrOK) return err;

    pSSD1306Handle->DeviceState.ScrollState.Direction = Direction;
    pSSD1306Handle->DeviceState.ScrollState.StartPageAddress = StartPage;
    pSSD1306Handle->DeviceState.ScrollState.EndPageAddress = EndPage;
    pSSD1306Handle->DeviceState.ScrollState.ScrollStepInterval = StepInterval;
    return err;
}

/**
 * @brief Setups the display for vertical and horizontal scrolling
 * @param pSSD1306Handle Device handle
 * @param Direction The desired scroll direction
 * @param StartPage The desired start page
 * @param EndPage The desired end page
 * @param StepInterval The time interval between each scroll step in terms of frame frequency
 * @param VerticalOffset Vertical scrolling offset
 */
SSD1306_Error_e SSD1306_SetupVerticalHorizontalScroll(SSD1306_Handle_t *pSSD1306Handle,
                                                      SSD1306_ScrollDirection_e Direction, SSD1306_Page_e StartPage, SSD1306_Page_e EndPage,
                                                      SSD1306_ScrollStepInterval_e StepInterval, uint8_t VerticalOffset) {
    if (Direction < SSD1306_VHScrollLeft || EndPage < StartPage || VerticalOffset < SSD1306_MIN_V_OFFSET || VerticalOffset > SSD1306_MAX_V_OFFSET) return SSD1306_ErrInvalidArg;
    SSD1306_Error_e err = SSD1306_ErrOK;

    uint8_t dirCommand = Direction == SSD1306_VHScrollLeft ? SSD1306_CMD_SET_CONT_VH_LEFT_SCROLLING_SETUP : SSD1306_CMD_SET_CONT_VH_RIGHT_SCROLLING_SETUP;

    SSD1306_SEND_COMMANDS(pSSD1306Handle, err, dirCommand, 0x00, StartPage, StepInterval, EndPage, VerticalOffset);
    if (err != SSD1306_ErrOK) return err;

    pSSD1306Handle->DeviceState.ScrollState.Direction = Direction;
    pSSD1306Handle->DeviceState.ScrollState.StartPageAddress = StartPage;
    pSSD1306Handle->DeviceState.ScrollState.EndPageAddress = EndPage;
    pSSD1306Handle->DeviceState.ScrollState.ScrollStepInterval = StepInterval;
    pSSD1306Handle->DeviceState.ScrollState.VerticalOffset = VerticalOffset;

    return err;
}

/**
 * @brief Disables the scrolling
 * @param pSSD1306Handle Device handle
 */
SSD1306_Error_e SSD1306_DeactivateScroll(SSD1306_Handle_t *pSSD1306Handle) {
    SSD1306_Error_e err = SSD1306_ErrOK;

    SSD1306_SEND_COMMANDS(pSSD1306Handle, err, SSD1306_CMD_SET_SCROLL_DISABLE);
    if (err != SSD1306_ErrOK) return err;

    pSSD1306Handle->DeviceState.ScrollState.Enabled = DISABLE;
    return err;
}

/**
 * @brief Enables the scrolling
 * @param pSSD1306Handle Device handle
 */
SSD1306_Error_e SSD1306_ActivateScroll(SSD1306_Handle_t *pSSD1306Handle) {
    SSD1306_Error_e err = SSD1306_ErrOK;

    SSD1306_SEND_COMMANDS(pSSD1306Handle, err, SSD1306_CMD_SET_SCROLL_ENABLE);
    if (err != SSD1306_ErrOK) return err;

    pSSD1306Handle->DeviceState.ScrollState.Enabled = ENABLE;
    return err;
}

/**
 * @brief Sets the area which should be scrolled
 * @param pSSD1306Handle Device handle
 * @param StartRow The start row from which the display scrolls
 * @param NumberOfRows The number of rows bellow the start row
 */
SSD1306_Error_e SSD1306_SetVerticalScrollArea(SSD1306_Handle_t *pSSD1306Handle, uint8_t StartRow, uint8_t NumberOfRows) {
    if (StartRow + NumberOfRows > pSSD1306Handle->DeviceState.HardwareState.MuxRatio) return SSD1306_ErrInvalidArg;
    if (NumberOfRows > pSSD1306Handle->DeviceState.HardwareState.MuxRatio) return SSD1306_ErrInvalidArg;
    if (pSSD1306Handle->DeviceState.ScrollState.VerticalOffset < NumberOfRows) return SSD1306_ErrInvalidArg;

    SSD1306_Error_e err = SSD1306_ErrOK;

    SSD1306_SEND_COMMANDS(pSSD1306Handle, err, SSD1306_CMD_SET_V_SCROLL_AREA, StartRow, NumberOfRows);
    if (err != SSD1306_ErrOK) return err;

    pSSD1306Handle->DeviceState.ScrollState.StartRow = StartRow;
    pSSD1306Handle->DeviceState.ScrollState.NumberOfRows = NumberOfRows;
    return err;
}

/**
 * @brief Sets the lower nibble of the column start address
 * @note This method is valid ONLY when the device is in PAGE ADDRESSING MODE
 * @param pSSD1306Handle Device handle
 * @param ColLowerAddr The lower nibble
 */
SSD1306_Error_e SSD1306_SetPageModeColumnLowerNibble(SSD1306_Handle_t *pSSD1306Handle, uint8_t ColLowerAddr) {
    if (ColLowerAddr < SSD1306_MIN_PAGE_LWR_COL_ADDR || ColLowerAddr > SSD1306_MAX_PAGE_LWR_COL_ADDR) return SSD1306_ErrInvalidArg;
    SSD1306_Error_e err = SSD1306_ErrOK;

    SSD1306_SEND_COMMANDS(pSSD1306Handle, err, ColLowerAddr);
    if (err != SSD1306_ErrOK) return err;

    pSSD1306Handle->DeviceState.AddressingState.ColLowerNibble = ColLowerAddr;
    return err;
}

/**
 * @brief Sets the higher nibble of the column start address
 * @note This method is valid ONLY when the device is in PAGE ADDRESSING MODE
 * @param pSSD1306Handle Device handle
 * @param ColHigherAddr The higher nibble
 */
SSD1306_Error_e SSD1306_SetPageModeColumnHigherNibble(SSD1306_Handle_t *pSSD1306Handle, uint8_t ColHigherAddr) {
    if (ColHigherAddr < SSD1306_MIN_PAGE_HIGH_COL_ADDR || ColHigherAddr > SSD1306_MAX_PAGE_HIGH_COL_ADDR) return SSD1306_ErrInvalidArg;
    SSD1306_Error_e err = SSD1306_ErrOK;

    SSD1306_SEND_COMMANDS(pSSD1306Handle, err, ColHigherAddr);
    if (err != SSD1306_ErrOK) return err;

    pSSD1306Handle->DeviceState.AddressingState.ColHigherNibble = ColHigherAddr;
    return err;
}

/**
 * @brief Sets the start page address
 * @note This method is valid ONLY when the device is in PAGE ADDRESSING MODE
 * @param pSSD1306Handle Device handle
 * @param Page The desired start page
 */
SSD1306_Error_e SSD1306_SetPageModeStartPage(SSD1306_Handle_t *pSSD1306Handle, SSD1306_Page_e Page) {
    SSD1306_Error_e err = SSD1306_ErrOK;

    SSD1306_SEND_COMMANDS(pSSD1306Handle, err, 0xB0 | Page);
    if (err != SSD1306_ErrOK) return err;

    pSSD1306Handle->DeviceState.AddressingState.PageStartAddr = Page;
    return err;
}

/**
 * @brief Sets the memory address mode
 * @param pSSD1306Handle Device handle
 * @param Mode The desired page addressing mode
 */
SSD1306_Error_e SSD1306_SetMemoryAddrMode(SSD1306_Handle_t *pSSD1306Handle, SSD1306_MemoryAddressingMode_e Mode) {
    SSD1306_Error_e err = SSD1306_ErrOK;

    SSD1306_SEND_COMMANDS(pSSD1306Handle, err, SSD1306_CMD_SET_MEM_ADDR_MODE, Mode);
    if (err != SSD1306_ErrOK) return err;

    pSSD1306Handle->DeviceState.AddressingState.AddressingMode = Mode;
    return err;
}

/**
 * @brief Sets the column start and end addresses
 * @note This method is valid ONLY when the device is in HORIZONTAL OR VERTICAL ACCESSING MODE
 * @param pSSD1306Handle Device handle
 * @param StartAddr The desired column start address
 * @param EndAddr The desired column end address
 */
SSD1306_Error_e SSD1306_SetHVModeColumnAddr(SSD1306_Handle_t *pSSD1306Handle, uint8_t StartAddr, uint8_t EndAddr) {
    if (StartAddr < SSD1306_MIN_COL_ADDR || StartAddr > SSD1306_MAX_COL_ADDR || EndAddr < SSD1306_MIN_COL_ADDR || EndAddr > SSD1306_MAX_COL_ADDR) {
        return SSD1306_ErrInvalidArg;
    }
    SSD1306_Error_e err = SSD1306_ErrOK;

    SSD1306_SEND_COMMANDS(pSSD1306Handle, err, SSD1306_CMD_SET_COL_ADDR, StartAddr, EndAddr);
    if (err != SSD1306_ErrOK) return err;

    pSSD1306Handle->DeviceState.AddressingState.ColStartAddr = StartAddr;
    pSSD1306Handle->DeviceState.AddressingState.ColEndAddr = EndAddr;
    return err;
}

/**
 * @brief Sets the desired start and end page addresses
 * @note This method is valid ONLY when the device is in HORIZONTAL OR VERTICAL ACCESSING MODE
 * @param pSSD1306Handle Device handle
 * @param StartPage The desired start page
 * @param EndPage The desired end page
 */
SSD1306_Error_e SSD1306_SetHVModePageAddr(SSD1306_Handle_t *pSSD1306Handle, SSD1306_Page_e StartPage,
                                          SSD1306_Page_e EndPage) {
    SSD1306_Error_e err = SSD1306_ErrOK;

    SSD1306_SEND_COMMANDS(pSSD1306Handle, err, SSD1306_CMD_SET_PAGE_ADDR,StartPage, EndPage);
    if (err != SSD1306_ErrOK) return err;

    pSSD1306Handle->DeviceState.AddressingState.PageStartAddr = StartPage;
    pSSD1306Handle->DeviceState.AddressingState.PageEndAddr = EndPage;
    return err;
}

/**
 * @brief Sets the start line of the display
 * @param pSSD1306Handle Device handle
 * @param StartLine The desired start line
 */
SSD1306_Error_e SSD1306_SetStartLine(SSD1306_Handle_t *pSSD1306Handle, uint8_t StartLine) {
    if (StartLine < SSD1306_DISPLAY_STRT_LINE_MIN || StartLine > SSD1306_DISPLAY_STRT_LINE_MAX) return SSD1306_ErrInvalidArg;
    SSD1306_Error_e err = SSD1306_ErrOK;

    SSD1306_SEND_COMMANDS(pSSD1306Handle, err, (0x40 | StartLine));
    if (err != SSD1306_ErrOK) return err;

    pSSD1306Handle->DeviceState.HardwareState.StartLine = StartLine;
    return err;
}

/**
 * @brief Controls whether the display segments should be remapped so that col 127 is mapped to SEG0
 * @param pSSD1306Handle Device handle
 * @param Enabled If the segments should be remapped
 */
SSD1306_Error_e SSD1306_SetSegmentRemap(SSD1306_Handle_t *pSSD1306Handle, uint8_t Enabled) {
    SSD1306_Error_e err = SSD1306_ErrOK;

    SSD1306_SEND_COMMANDS(pSSD1306Handle, err, Enabled ? SSD1306_CMD_SET_REMAP_START_127 : SSD1306_CMD_SET_REMAP_START_0);
    if (err != SSD1306_ErrOK) return err;

    pSSD1306Handle->DeviceState.HardwareState.SegmentRemapped = Enabled;
    return err;
}

/**
 * @brief Controls how much rows should be visible
 * @param pSSD1306Handle Device handle
 * @param Ratio Ratio between 15 and 63
 */
SSD1306_Error_e SSD1306_SetMuxRatio(SSD1306_Handle_t *pSSD1306Handle, uint8_t Ratio) {
    if (Ratio < SSD1306_MIN_MUX_RATIO || Ratio > SSD1306_MAX_MUX_RATIO) return SSD1306_ErrInvalidArg;
    SSD1306_Error_e err = SSD1306_ErrOK;

    SSD1306_SEND_COMMANDS(pSSD1306Handle, err, SSD1306_CMD_SET_MUX_RATIO, Ratio);
    if (err != SSD1306_ErrOK) return err;

    pSSD1306Handle->DeviceState.HardwareState.MuxRatio = Ratio;
    return err;
}

/**
 * @brief Controls the com output scan direction
 * @param pSSD1306Handle Device handle
 * @param Enabled If the reverse com scan direction should be enabled
 */
SSD1306_Error_e SSD1306_SetCOMScanRemapping(SSD1306_Handle_t *pSSD1306Handle, uint8_t Enabled) {
    SSD1306_Error_e err = SSD1306_ErrOK;

    SSD1306_SEND_COMMANDS(pSSD1306Handle, err, Enabled ? SSD1306_CMD_SET_COM_SCAN_DIR_REVERSE : SSD1306_CMD_SET_COM_SCAN_DIR_NORMAL);
    if (err != SSD1306_ErrOK) return err;

    pSSD1306Handle->DeviceState.HardwareState.COMScanRemapped = Enabled;
    return err;
}

/**
 * @brief Controls the display vertical offset§
 * @param pSSD1306Handle Device handle
 * @param Value The desired vertical offset
 */
SSD1306_Error_e SSD1306_SetDisplayOffset(SSD1306_Handle_t *pSSD1306Handle, uint8_t Value) {
    if (Value < SSD1306_MIN_V_OFFSET || Value > SSD1306_MAX_V_OFFSET) return SSD1306_ErrInvalidArg;
    SSD1306_Error_e err = SSD1306_ErrOK;

    SSD1306_SEND_COMMANDS(pSSD1306Handle, err, SSD1306_CMD_SET_DISPLAY_OFFSET, Value);
    if (err != SSD1306_ErrOK) return err;

    pSSD1306Handle->DeviceState.HardwareState.DisplayVOffset = Value;
    return err;
}

/**
 * @brief Controls the COM pin configuration and the Left/Right remap
 * @param pSSD1306Handle Device handle
 * @param AlternativeCOMPinConfigEnabled If the alternative COM pin configuration should be enabled
 * @param COMLeftRightRemapEnabled If the COM Left/Right remap should be enabled
 * @return
 */
SSD1306_Error_e SSD1306_ConfigureCOMPins(SSD1306_Handle_t *pSSD1306Handle, uint8_t AlternativeCOMPinConfigEnabled,
                                         uint8_t COMLeftRightRemapEnabled) {
    SSD1306_Error_e err = SSD1306_ErrOK;

    uint8_t finalCommand = (AlternativeCOMPinConfigEnabled ? 1 : 0) << 4;
    finalCommand |= (COMLeftRightRemapEnabled ? 1 : 0) << 5;

    SSD1306_SEND_COMMANDS(pSSD1306Handle, err, SSD1306_CMD_SET_COM_PINS_HDWR_CONF, finalCommand);
    if (err != SSD1306_ErrOK) return err;

    pSSD1306Handle->DeviceState.HardwareState.AlternativeCOMPinConfig = AlternativeCOMPinConfigEnabled;
    pSSD1306Handle->DeviceState.HardwareState.COMLeftRightRemapEnabled = COMLeftRightRemapEnabled;
    return err;
}

/**
 * @brief Controls the display clock division and source frequency
 * @param pSSD1306Handle Device handle
 * @param DivideRatio The desired display clock divide ratio
 * @param OSCFreq Source oscillator frequency
 */
SSD1306_Error_e SSD1306_SetDivideRatioAndOSCFreq(SSD1306_Handle_t *pSSD1306Handle, uint8_t DivideRatio, uint8_t OSCFreq) {
    if (DivideRatio < SSD1306_MIN_DCLK_DIVIDE_RATIO || DivideRatio > SSD1306_MAX_DCLK_DIVIDE_RATIO || OSCFreq < SSD1306_MIN_OSC_FREQ || OSCFreq > SSD1306_MAX_OSC_FREQ) {
        return SSD1306_ErrInvalidArg;
    }

    SSD1306_Error_e err = SSD1306_ErrOK;

    uint8_t finalCommand = (DivideRatio & 0xF) | ((OSCFreq & 0xF) << 4);

    SSD1306_SEND_COMMANDS(pSSD1306Handle, err, SSD1306_CMD_SET_DCLK_DIV_RATIO_OSC_FREQ, finalCommand);

    if (err != SSD1306_ErrOK) return err;

    pSSD1306Handle->DeviceState.TimingState.DCLKDivRatio = DivideRatio;
    pSSD1306Handle->DeviceState.TimingState.OSCFreq = OSCFreq;
    return err;
}

/**
 * @brief Sets the desired pre-charge phase periods
 * @param pSSD1306Handle Device handle
 * @param PhaseOnePeriod The DCLK period of the first pre-charge phase
 * @param PhaseTwoPeriod The DCLK period of the second pre-charge phase
 */
SSD1306_Error_e SSD1306_SetPreChargePeriod(SSD1306_Handle_t *pSSD1306Handle, uint8_t PhaseOnePeriod,
                                           uint8_t PhaseTwoPeriod) {
    if (PhaseOnePeriod < SSD1306_MIN_PHASE_PRECHARGE_PERIOD || PhaseOnePeriod > SSD1306_MAX_PHASE_PRECHARGE_PERIOD || PhaseTwoPeriod < SSD1306_MIN_PHASE_PRECHARGE_PERIOD || PhaseTwoPeriod > SSD1306_MAX_PHASE_PRECHARGE_PERIOD) {
        return SSD1306_ErrInvalidArg;
    }

    SSD1306_Error_e err = SSD1306_ErrOK;

    uint8_t finalCommand = (PhaseOnePeriod & 0xF) | ((PhaseTwoPeriod & 0xF) << 4);

    SSD1306_SEND_COMMANDS(pSSD1306Handle, err, SSD1306_CMD_SET_PRE_CHARGE_PERIOD, finalCommand);

    if (err != SSD1306_ErrOK) return err;

    pSSD1306Handle->DeviceState.TimingState.Phase1Period = PhaseOnePeriod;
    pSSD1306Handle->DeviceState.TimingState.Phase2Period = PhaseTwoPeriod;
    return err;
}

/**
 * @brief Controls the VCOMH deselect level
 * @param pSSD1306Handle Device handle
 * @param Level The desired VCOMH deselect level
 */
SSD1306_Error_e SSD1306_SetVCOMHDeselectLevel(SSD1306_Handle_t *pSSD1306Handle, SSD1306_VCOMHDeselectLevel_e Level) {
    SSD1306_Error_e err = SSD1306_ErrOK;

    SSD1306_SEND_COMMANDS(pSSD1306Handle, err, SSD1306_CMD_SET_VCOMH_DES_LVL, (Level & 0x07) << 4);
    if (err != SSD1306_ErrOK) return err;

    pSSD1306Handle->DeviceState.TimingState.VCOMHDeselectLevel = Level;
    return err;
}

SSD1306_Error_e send_commands(SSD1306_Handle_t *pSSD1306Handle, uint8_t *commands, uint8_t len) {
    if (!pSSD1306Handle->DeviceInitialized) return SSD1306_ErrNotInitialized;
    if (len + 1 > SSD1306_I2C_BUFFER_MAX_LEN) return SSD1306_ErrBufferOverflow;
    if (len == 0) return SSD1306_ErrOK;

    I2C_Error_e err = I2C_ErrOK;

    memset(buffer, 0, sizeof(buffer));
    I2C_PeripheralControl(pSSD1306Handle->Config.I2CHandle->pI2Cx, ENABLE);

    buffer[0] = 0x00;                       // Co == 0 && D/C# == 0
    memcpy(buffer + 1, commands, len);

    err = I2C_MasterSendData(pSSD1306Handle->Config.I2CHandle, buffer, len + 1, SSD1306_I2C_ADDR, I2C_StopEnabled);

    I2C_PeripheralControl(pSSD1306Handle->Config.I2CHandle->pI2Cx, DISABLE);
    return err == I2C_ErrOK ? SSD1306_ErrOK : SSD1306_ErrComm;
}

SSD1306_Error_e send_data(SSD1306_Handle_t *pSSD1306Handle, uint8_t *data, uint8_t len) {
    if (!pSSD1306Handle->DeviceInitialized) return SSD1306_ErrNotInitialized;
    if (len + 1 > SSD1306_I2C_BUFFER_MAX_LEN) return SSD1306_ErrBufferOverflow;
    I2C_Error_e err = I2C_ErrOK;

    I2C_PeripheralControl(pSSD1306Handle->Config.I2CHandle->pI2Cx, ENABLE);

    memset(buffer, 0, sizeof(buffer));

    buffer[0] = (1 << SSD1306_COMM_DC_BIT_POS);      // Co == 0 && D/C# == 1
    memcpy(buffer + 1, data, len);

    err = I2C_MasterSendData(pSSD1306Handle->Config.I2CHandle, buffer, len + 1, SSD1306_I2C_ADDR, I2C_StopEnabled);

    I2C_PeripheralControl(pSSD1306Handle->Config.I2CHandle->pI2Cx, DISABLE);
    return err == I2C_ErrOK ? SSD1306_ErrOK : SSD1306_ErrComm;
}

void reset_state(SSD1306_Handle_t *pSSD1306Handle) {
    pSSD1306Handle->DeviceState.Contrast = 0x7F;
    pSSD1306Handle->DeviceState.EntireDisplayON = DISABLE;
    pSSD1306Handle->DeviceState.DisplayInversed = DISABLE;
    pSSD1306Handle->DeviceState.DisplayMode = SSD1306_DisplayModeSleep;

    pSSD1306Handle->DeviceState.ScrollState.Direction = SSD1306_HScrollRight;
    pSSD1306Handle->DeviceState.ScrollState.StartPageAddress = SSD1306_Page0;
    pSSD1306Handle->DeviceState.ScrollState.ScrollStepInterval = SSD1306_ScrollInterval5Frames;
    pSSD1306Handle->DeviceState.ScrollState.EndPageAddress = SSD1306_Page0;
    pSSD1306Handle->DeviceState.ScrollState.VerticalOffset = SSD1306_MIN_V_OFFSET;
    pSSD1306Handle->DeviceState.ScrollState.Enabled = DISABLE;
    pSSD1306Handle->DeviceState.ScrollState.StartRow = 0x00;
    pSSD1306Handle->DeviceState.ScrollState.NumberOfRows = 0x40;

    pSSD1306Handle->DeviceState.AddressingState.ColLowerNibble = SSD1306_MIN_PAGE_LWR_COL_ADDR;
    pSSD1306Handle->DeviceState.AddressingState.ColHigherNibble = SSD1306_MIN_PAGE_HIGH_COL_ADDR;
    pSSD1306Handle->DeviceState.AddressingState.AddressingMode = SSD1306_MemAddrPage;
    pSSD1306Handle->DeviceState.AddressingState.ColStartAddr = SSD1306_MIN_COL_ADDR;
    pSSD1306Handle->DeviceState.AddressingState.ColEndAddr = SSD1306_MAX_COL_ADDR;
    pSSD1306Handle->DeviceState.AddressingState.PageStartAddr = SSD1306_Page0;
    pSSD1306Handle->DeviceState.AddressingState.PageEndAddr = SSD1306_Page7;

    pSSD1306Handle->DeviceState.HardwareState.StartLine = SSD1306_DISPLAY_STRT_LINE_MIN;
    pSSD1306Handle->DeviceState.HardwareState.SegmentRemapped = DISABLE;
    pSSD1306Handle->DeviceState.HardwareState.MuxRatio = SSD1306_MAX_MUX_RATIO;
    pSSD1306Handle->DeviceState.HardwareState.COMScanRemapped = DISABLE;
    pSSD1306Handle->DeviceState.HardwareState.DisplayVOffset = SSD1306_MIN_V_OFFSET;
    pSSD1306Handle->DeviceState.HardwareState.AlternativeCOMPinConfig = ENABLE;
    pSSD1306Handle->DeviceState.HardwareState.COMLeftRightRemapEnabled = DISABLE;

    pSSD1306Handle->DeviceState.TimingState.DCLKDivRatio = SSD1306_MIN_DCLK_DIVIDE_RATIO;
    pSSD1306Handle->DeviceState.TimingState.OSCFreq = 0x08;
    pSSD1306Handle->DeviceState.TimingState.Phase1Period = 0x02;
    pSSD1306Handle->DeviceState.TimingState.Phase2Period = 0x02;
    pSSD1306Handle->DeviceState.TimingState.VCOMHDeselectLevel = SSD1306_VCOMHLevelMode1;
}
