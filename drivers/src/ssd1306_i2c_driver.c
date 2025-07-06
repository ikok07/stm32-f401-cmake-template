//
// Created by Kok on 7/2/25.
//

#include "ssd1306_i2c_driver.h"

#include <string.h>
#include "commons.h"

static uint8_t sendBuffer[SSD1306_I2C_SEND_BUFFER_LEN];
static uint8_t frameBuffer[SSD1306_I2C_SEND_BUFFER_LEN - 1];            // Omitting the control byte

static SSD1306_Error_e send_commands(SSD1306_Handle_t *pSSD1306Handle, uint8_t *commands, uint32_t len);
static SSD1306_Error_e send_data(SSD1306_Handle_t *pSSD1306Handle, uint8_t *data, uint32_t len);
static void h_addr_set_pixel(uint8_t x, uint8_t y, uint8_t on);
static void v_addr_set_pixel(uint8_t x, uint8_t y, uint8_t on);
static uint8_t *get_char_bitmap(char c);
static void reset_state(SSD1306_Handle_t *pSSD1306Handle);

#define SSD1306_SEND_COMMANDS(handle, err, ...)         do {                                                        \
                                                            uint8_t cmds[] = {__VA_ARGS__};                         \
                                                            err = send_commands(handle, cmds, sizeof(cmds));        \
                                                        } while (0)

static uint8_t font6x8[96][6] = {
    {0x00,0x00,0x00,0x00,0x00,0x00}, // ' '
    {0x00,0x00,0x5F,0x00,0x00,0x00}, // '!'
    {0x00,0x07,0x00,0x07,0x00,0x00}, // '"'
    {0x14,0x7F,0x14,0x7F,0x14,0x00}, // '#'
    {0x24,0x2A,0x7F,0x2A,0x12,0x00}, // '$'
    {0x23,0x13,0x08,0x64,0x62,0x00}, // '%'
    {0x36,0x49,0x55,0x22,0x50,0x00}, // '&'
    {0x00,0x05,0x03,0x00,0x00,0x00}, // '''
    {0x00,0x1C,0x22,0x41,0x00,0x00}, // '('
    {0x00,0x41,0x22,0x1C,0x00,0x00}, // ')'
    {0x14,0x08,0x3E,0x08,0x14,0x00}, // '*'
    {0x08,0x08,0x3E,0x08,0x08,0x00}, // '+'
    {0x00,0x50,0x30,0x00,0x00,0x00}, // ','
    {0x08,0x08,0x08,0x08,0x08,0x00}, // '-'
    {0x00,0x60,0x60,0x00,0x00,0x00}, // '.'
    {0x20,0x10,0x08,0x04,0x02,0x00}, // '/'
    {0x3E,0x51,0x49,0x45,0x3E,0x00}, // '0'
    {0x00,0x42,0x7F,0x40,0x00,0x00}, // '1'
    {0x42,0x61,0x51,0x49,0x46,0x00}, // '2'
    {0x21,0x41,0x45,0x4B,0x31,0x00}, // '3'
    {0x18,0x14,0x12,0x7F,0x10,0x00}, // '4'
    {0x27,0x45,0x45,0x45,0x39,0x00}, // '5'
    {0x3C,0x4A,0x49,0x49,0x30,0x00}, // '6'
    {0x01,0x71,0x09,0x05,0x03,0x00}, // '7'
    {0x36,0x49,0x49,0x49,0x36,0x00}, // '8'
    {0x06,0x49,0x49,0x29,0x1E,0x00}, // '9'
    {0x00,0x36,0x36,0x00,0x00,0x00}, // ':'
    {0x00,0x56,0x36,0x00,0x00,0x00}, // ';'
    {0x08,0x14,0x22,0x41,0x00,0x00}, // '<'
    {0x14,0x14,0x14,0x14,0x14,0x00}, // '='
    {0x00,0x41,0x22,0x14,0x08,0x00}, // '>'
    {0x02,0x01,0x51,0x09,0x06,0x00}, // '?'
    {0x32,0x49,0x79,0x41,0x3E,0x00}, // '@'
    {0x7E,0x09,0x09,0x09,0x7E,0x00}, // 'A'
    {0x7F,0x49,0x49,0x49,0x36,0x00}, // 'B'
    {0x3E,0x41,0x41,0x41,0x22,0x00}, // 'C'
    {0x7F,0x41,0x41,0x22,0x1C,0x00}, // 'D'
    {0x7F,0x49,0x49,0x49,0x41,0x00}, // 'E'
    {0x7F,0x09,0x09,0x09,0x01,0x00}, // 'F'
    {0x3E,0x41,0x49,0x49,0x7A,0x00}, // 'G'
    {0x7F,0x08,0x08,0x08,0x7F,0x00}, // 'H'
    {0x00,0x41,0x7F,0x41,0x00,0x00}, // 'I'
    {0x20,0x40,0x41,0x3F,0x01,0x00}, // 'J'
    {0x7F,0x08,0x14,0x22,0x41,0x00}, // 'K'
    {0x7F,0x40,0x40,0x40,0x40,0x00}, // 'L'
    {0x7F,0x02,0x0C,0x02,0x7F,0x00}, // 'M'
    {0x7F,0x04,0x08,0x10,0x7F,0x00}, // 'N'
    {0x3E,0x41,0x41,0x41,0x3E,0x00}, // 'O'
    {0x7F,0x09,0x09,0x09,0x06,0x00}, // 'P'
    {0x3E,0x41,0x51,0x21,0x5E,0x00}, // 'Q'
    {0x7F,0x09,0x19,0x29,0x46,0x00}, // 'R'
    {0x46,0x49,0x49,0x49,0x31,0x00}, // 'S'
    {0x01,0x01,0x7F,0x01,0x01,0x00}, // 'T'
    {0x3F,0x40,0x40,0x40,0x3F,0x00}, // 'U'
    {0x1F,0x20,0x40,0x20,0x1F,0x00}, // 'V'
    {0x3F,0x40,0x38,0x40,0x3F,0x00}, // 'W'
    {0x63,0x14,0x08,0x14,0x63,0x00}, // 'X'
    {0x07,0x08,0x70,0x08,0x07,0x00}, // 'Y'
    {0x61,0x51,0x49,0x45,0x43,0x00}, // 'Z'
    {0x00,0x7F,0x41,0x41,0x00,0x00}, // '['
    {0x02,0x04,0x08,0x10,0x20,0x00}, // '\'
    {0x00,0x41,0x41,0x7F,0x00,0x00}, // ']'
    {0x04,0x02,0x01,0x02,0x04,0x00}, // '^'
    {0x80,0x80,0x80,0x80,0x80,0x00}, // '_'
    {0x00,0x03,0x05,0x00,0x00,0x00}, // '`'
    {0x20,0x54,0x54,0x54,0x78,0x00}, // 'a'
    {0x7F,0x48,0x44,0x44,0x38,0x00}, // 'b'
    {0x38,0x44,0x44,0x44,0x20,0x00}, // 'c'
    {0x38,0x44,0x44,0x48,0x7F,0x00}, // 'd'
    {0x38,0x54,0x54,0x54,0x18,0x00}, // 'e'
    {0x08,0x7E,0x09,0x01,0x02,0x00}, // 'f'
    {0x0C,0x52,0x52,0x52,0x3E,0x00}, // 'g'
    {0x7F,0x08,0x04,0x04,0x78,0x00}, // 'h'
    {0x00,0x44,0x7D,0x40,0x00,0x00}, // 'i'
    {0x20,0x40,0x44,0x3D,0x00,0x00}, // 'j'
    {0x7F,0x10,0x28,0x44,0x00,0x00}, // 'k'
    {0x00,0x41,0x7F,0x40,0x00,0x00}, // 'l'
    {0x7C,0x04,0x18,0x04,0x78,0x00}, // 'm'
    {0x7C,0x08,0x04,0x04,0x78,0x00}, // 'n'
    {0x38,0x44,0x44,0x44,0x38,0x00}, // 'o'
    {0x7C,0x14,0x14,0x14,0x08,0x00}, // 'p'
    {0x08,0x14,0x14,0x18,0x7C,0x00}, // 'q'
    {0x7C,0x08,0x04,0x04,0x08,0x00}, // 'r'
    {0x48,0x54,0x54,0x54,0x20,0x00}, // 's'
    {0x04,0x3F,0x44,0x40,0x20,0x00}, // 't'
    {0x3C,0x40,0x40,0x20,0x7C,0x00}, // 'u'
    {0x1C,0x20,0x40,0x20,0x1C,0x00}, // 'v'
    {0x3C,0x40,0x30,0x40,0x3C,0x00}, // 'w'
    {0x44,0x28,0x10,0x28,0x44,0x00}, // 'x'
    {0x0C,0x50,0x50,0x50,0x3C,0x00}, // 'y'
    {0x44,0x64,0x54,0x4C,0x44,0x00}, // 'z'
    {0x00,0x08,0x36,0x41,0x00,0x00}, // '{'
    {0x00,0x00,0x7F,0x00,0x00,0x00}, // '|'
    {0x00,0x41,0x36,0x08,0x00,0x00}, // '}'
    {0x02,0x01,0x02,0x04,0x02,0x00}, // '~'
};

/**
 * @brief Sets the display cursor in PAGE ADDRESSING MODE
 * @param pSSD1306Handle Device handle
 * @param Column The desired column (X Axis)
 * @param Page The desired page (Y Axis)
 */
SSD1306_Error_e SSD1306_SetCursor(SSD1306_Handle_t *pSSD1306Handle, uint8_t Column, SSD1306_Page_e Page) {
    if (Column < SSD1306_MIN_COL_ADDR || Column > SSD1306_MAX_COL_ADDR) return SSD1306_ErrInvalidArg;
    if (pSSD1306Handle->DeviceState.AddressingState.AddressingMode != SSD1306_MemAddrPage) return SSD1306_ErrInvalidAddrMode;

    SSD1306_Error_e err = SSD1306_ErrOK;

    // X Axis
    uint8_t lowerNibble = Column & 0xF;
    uint8_t higherNibble = (Column >> 4) & 0xF;
    if ((err = SSD1306_SetPageModeColumnLowerNibble(pSSD1306Handle, lowerNibble)) != SSD1306_ErrOK) return err;
    if ((err = SSD1306_SetPageModeColumnHigherNibble(pSSD1306Handle, higherNibble)) != SSD1306_ErrOK) return err;

    // Y Axis
    if ((err = SSD1306_SetPageModeStartPage(pSSD1306Handle, Page)) != SSD1306_ErrOK) return err;

    return err;
}

/**
 * @brief Writes a string on the display when in PAGE ADDRESSING MODE
 * @param pSSD1306Handle Device handle
 * @param str String to be written
 */
SSD1306_Error_e SSD1306_Write(SSD1306_Handle_t *pSSD1306Handle, char *str) {
    if (pSSD1306Handle->DeviceState.AddressingState.AddressingMode != SSD1306_MemAddrPage) return SSD1306_ErrInvalidAddrMode;

    SSD1306_Error_e err = SSD1306_ErrOK;

    while (*str) {
        uint8_t *fontChar = get_char_bitmap(*str++);
        if ((err = send_data(pSSD1306Handle, fontChar, SSD1306_FONT_WIDTH)) != SSD1306_ErrOK) return err;
    }

    return err;
}

/**
 * @brief Clear the display in PAGE ADDRESSING MODE
 * @param pSSD1306Handle Device handle
 */
SSD1306_Error_e SSD1306_Clear(SSD1306_Handle_t *pSSD1306Handle) {
    if (pSSD1306Handle->DeviceState.AddressingState.AddressingMode != SSD1306_MemAddrPage) return SSD1306_ErrInvalidAddrMode;

    SSD1306_Error_e err = SSD1306_ErrOK;

    uint8_t emptyLine[SSD1306_MAX_WIDTH] = {0};

    for (int i = 0; i <= SSD1306_MAX_PAGE_ADDR; i++) {
        if ((err = SSD1306_SetPageModeStartPage(pSSD1306Handle, i)) != SSD1306_ErrOK) return err;
        if ((err = SSD1306_SetPageModeColumnLowerNibble(pSSD1306Handle, 0)) != SSD1306_ErrOK) return err;
        if ((err = SSD1306_SetPageModeColumnHigherNibble(pSSD1306Handle, 0)) != SSD1306_ErrOK) return err;
        if ((err = send_data(pSSD1306Handle, emptyLine, sizeof(emptyLine))) != SSD1306_ErrOK) return err;
    }

    return err;
}

/**
 * @brief Sets the write area in HORIZONTAL / VERTICAL ADDRESSING MODE
 * @param pSSD1306Handle Device handle
 * @param xStart Desired start x position
 * @param xEnd Desired end x position
 * @param yStart Desired start y position
 * @param yEnd Desired end y position
 */
SSD1306_Error_e SSD1306_SetWriteAreaHV(SSD1306_Handle_t *pSSD1306Handle, uint8_t xStart, uint8_t xEnd, uint8_t yStart, uint8_t yEnd) {
    if (xStart < SSD1306_MIN_WIDTH || xEnd > SSD1306_MAX_WIDTH || yStart < SSD1306_MIN_HEIGHT || yEnd > SSD1306_MAX_HEIGHT) return SSD1306_ErrInvalidArg;
    if (xStart > xEnd || yStart > yEnd) return SSD1306_ErrInvalidArg;
    if (
        pSSD1306Handle->DeviceState.AddressingState.AddressingMode != SSD1306_MemAddrHorizontal &&
        pSSD1306Handle->DeviceState.AddressingState.AddressingMode != SSD1306_MemAddrVertical
    ) {
            return SSD1306_ErrInvalidAddrMode;
    }

    SSD1306_Error_e err = SSD1306_ErrOK;

    uint8_t pageStart = yStart / 8;
    uint8_t pageEnd = yEnd / 8;

    SSD1306_SetHVModeColumnAddr(pSSD1306Handle, xStart, xEnd);
    SSD1306_SetHVModePageAddr(pSSD1306Handle, pageStart, pageEnd);

    return err;
}

SSD1306_Error_e SSD1306_WriteHV(SSD1306_Handle_t *pSSD1306Handle, uint8_t x, uint8_t y, char *str) {
    if (
        pSSD1306Handle->DeviceState.AddressingState.AddressingMode != SSD1306_MemAddrHorizontal &&
        pSSD1306Handle->DeviceState.AddressingState.AddressingMode != SSD1306_MemAddrVertical
    ) {
        return SSD1306_ErrInvalidAddrMode;
    }

    SSD1306_Error_e err = SSD1306_ErrOK;

    if (x + strlen(str) * SSD1306_FONT_WIDTH > pSSD1306Handle->DeviceState.AddressingState.ColEndAddr || y + SSD1306_FONT_HEIGHT > pSSD1306Handle->DeviceState.AddressingState.PageEndAddr) {
        return SSD1306_ErrOutOfBounds;
    }

    while (*str) {
        uint8_t *fontChar = get_char_bitmap(*str++);
        for (uint8_t i = 0; i < SSD1306_FONT_WIDTH; i++) {
            uint8_t col = fontChar[i];
            for (uint8_t j = 0; j < SSD1306_FONT_HEIGHT; j++) {
                if (pSSD1306Handle->DeviceState.AddressingState.AddressingMode == SSD1306_MemAddrHorizontal) {
                    h_addr_set_pixel(x + i, y + j, (col >> j) & 0x01);
                } else {
                    v_addr_set_pixel(x + i, y + j, (col >> j) & 0x01);
                }
            }
        }
    }

    return err;
}

SSD1306_Error_e SSD1306_DrawHV(SSD1306_Handle_t *pSSD1306Handle, uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t *bitmap, uint8_t len) {
    if (
        pSSD1306Handle->DeviceState.AddressingState.AddressingMode != SSD1306_MemAddrHorizontal &&
        pSSD1306Handle->DeviceState.AddressingState.AddressingMode != SSD1306_MemAddrVertical
    ) {
        return SSD1306_ErrInvalidAddrMode;
    }

    SSD1306_Error_e err = SSD1306_ErrOK;
    uint8_t bytesPerColumn = (height + 7) / 8;

    if (x + width > pSSD1306Handle->DeviceState.AddressingState.ColEndAddr || y + height > pSSD1306Handle->DeviceState.AddressingState.PageEndAddr || width * bytesPerColumn > len) {
        return SSD1306_ErrOutOfBounds;
    }

    for (uint8_t col = 0; col < width; col++) {
        // Column
        for (int byteInCol = 0; byteInCol < bytesPerColumn; byteInCol++) {
            // Byte of column
            uint8_t colData = bitmap[col * bytesPerColumn + byteInCol];
            for (uint8_t bit = 0; bit < 8; bit++) {
                // Bit of the column byte
                uint8_t pixelRow = byteInCol * 8 + bit;
                if (pixelRow >= height) break;

                if (pSSD1306Handle->DeviceState.AddressingState.AddressingMode == SSD1306_MemAddrHorizontal) {
                    h_addr_set_pixel(x + col, y + pixelRow, (colData >> bit) & 0x01);
                } else {
                    v_addr_set_pixel(x + col, y + pixelRow, (colData >> bit) & 0x01);
                }
            }
        }
    }

    return err;
}

/**
 * @brief Clears the selected display area in HORIZONTAL / VERTICAL ADDRESSING MODE
 * @param pSSD1306Handle Device handle
 */
SSD1306_Error_e SSD1306_ClearAreaHV(SSD1306_Handle_t *pSSD1306Handle) {
    if (
        pSSD1306Handle->DeviceState.AddressingState.AddressingMode != SSD1306_MemAddrHorizontal &&
        pSSD1306Handle->DeviceState.AddressingState.AddressingMode != SSD1306_MemAddrVertical
    ) {
        return SSD1306_ErrInvalidAddrMode;
    }
    memset(frameBuffer, 0, sizeof(frameBuffer));

    return SSD1306_UpdateHV(pSSD1306Handle);
}

/**
 * @brief Updates the whole display in HORIZONTAL / VERTICAL ADDRESSING MODE
 * @param pSSD1306Handle Device handle
 */
SSD1306_Error_e SSD1306_UpdateHV(SSD1306_Handle_t *pSSD1306Handle) {
    if (
        pSSD1306Handle->DeviceState.AddressingState.AddressingMode != SSD1306_MemAddrHorizontal &&
        pSSD1306Handle->DeviceState.AddressingState.AddressingMode != SSD1306_MemAddrVertical
    ) {
        return SSD1306_ErrInvalidAddrMode;
    }

    return send_data(pSSD1306Handle, frameBuffer, sizeof(frameBuffer));
}

/**
 * @brief Initializes the device I2C communication
 * @note Before calling this method the required GPIOS \b MUST be configured.\n\n
 * @note Also the i2c handle inside the config structure \b SHOULD include pointer to the i2c peripheral
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

SSD1306_Error_e send_commands(SSD1306_Handle_t *pSSD1306Handle, uint8_t *commands, uint32_t len) {
    if (!pSSD1306Handle->DeviceInitialized) return SSD1306_ErrNotInitialized;
    if (len + 1 > SSD1306_I2C_SEND_BUFFER_LEN) return SSD1306_ErrBufferOverflow;
    if (len == 0) return SSD1306_ErrOK;

    I2C_Error_e err = I2C_ErrOK;

    memset(sendBuffer, 0, sizeof(sendBuffer));
    I2C_PeripheralControl(pSSD1306Handle->Config.I2CHandle->pI2Cx, ENABLE);

    sendBuffer[0] = 0x00;                       // Co == 0 && D/C# == 0
    memcpy(sendBuffer + 1, commands, len);

    err = I2C_MasterSendData(pSSD1306Handle->Config.I2CHandle, sendBuffer, len + 1, SSD1306_I2C_ADDR, I2C_StopEnabled);

    I2C_PeripheralControl(pSSD1306Handle->Config.I2CHandle->pI2Cx, DISABLE);
    return err == I2C_ErrOK ? SSD1306_ErrOK : SSD1306_ErrComm;
}

SSD1306_Error_e send_data(SSD1306_Handle_t *pSSD1306Handle, uint8_t *data, uint32_t len) {
    if (!pSSD1306Handle->DeviceInitialized) return SSD1306_ErrNotInitialized;
    if (len + 1 > SSD1306_I2C_SEND_BUFFER_LEN) return SSD1306_ErrBufferOverflow;
    I2C_Error_e err = I2C_ErrOK;

    I2C_PeripheralControl(pSSD1306Handle->Config.I2CHandle->pI2Cx, ENABLE);

    memset(sendBuffer, 0, sizeof(sendBuffer));

    sendBuffer[0] = (1 << SSD1306_COMM_DC_BIT_POS);      // Co == 0 && D/C# == 1
    memcpy(sendBuffer + 1, data, len);

    err = I2C_MasterSendData(pSSD1306Handle->Config.I2CHandle, sendBuffer, len + 1, SSD1306_I2C_ADDR, I2C_StopEnabled);

    I2C_PeripheralControl(pSSD1306Handle->Config.I2CHandle->pI2Cx, DISABLE);
    return err == I2C_ErrOK ? SSD1306_ErrOK : SSD1306_ErrComm;
}

void h_addr_set_pixel(uint8_t x, uint8_t y, uint8_t on) {
    uint8_t page = y / 8;
    uint8_t bit = y % 8;
    uint32_t index = page * (SSD1306_MAX_WIDTH + 1) + x;

    if (on) frameBuffer[index] |= (1 << bit);
    else frameBuffer[index] &=~ (1 << bit);
}

void v_addr_set_pixel(uint8_t x, uint8_t y, uint8_t on) {
    uint32_t colOffset = x * 8;
    uint8_t page = y / 8;
    uint8_t bit = y % 8;

    if (on) frameBuffer[colOffset + page] |= (1 << bit);
    else frameBuffer[colOffset + page] &=~ (1 << bit);
}

uint8_t *get_char_bitmap(char c) {
    if (c < 32 || c > 97) c = '?';
    return font6x8[c - 32];
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
