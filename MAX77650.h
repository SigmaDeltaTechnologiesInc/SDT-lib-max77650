/* MAX77650.h
 * 
 * Copyright (c) 2018 Sigma Delta Technologies Inc.
 * 
 * MIT License
 * 
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 * 
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef _MAX77650_H_
#define _MAX77650_H_

#include "mbed.h"

#define MAX77650_NO_ERROR       0
#define MAX77650_ERROR          -1

#define MAX77650_I2C_ADDRESS    0x90

#define MAX77650_CID            0x78

/**
 * @brief MAX77650 Ultra-Low Power PMIC with 3-Output SIMO and Charger
 *
 * @details The MAX77650/MAX77651 provide highly-integrated battery
 * charging and power supply solutions for low-power wearable applications
 * where size and efficiency are critical.
 * <br>https://www.maximintegrated.com/en/products/power/battery-management/MAX77650.html
 *
 * @code
 * #include "mbed.h"
 * #include "MAX77650.h"
 *
 * // Configure LED to rotate colors
 * static const char ledRotate[] = {
 *     MAX77650::CNFG_LED0_A,
 *     0x44, 0x44,  0x44,
 *     0x17, 0x37,  0x77,
 *     0x01,
 * };
 *
 * I2C i2c2(I2C2_SDA, I2C2_SCL);
 *
 * MAX77650 max77650(i2c2);
 *
 * int main()
 * {
 *     // Turn off Low-Dropout Linear Regulator
 *     max77650.disableLDO();
 *
 *     // Set SBB supply 2 at 3.3V
 *     max77650.setSBB2Voltage(3.3f);
 *
 *     // Configure LED current sinks
 *     max77650.writeReg(ledRotate, sizeof(ledRotate));
 *
 *     while(1) ;
 *
 * }
 * @endcode
 */

class MAX77650
{
public:

    /**
     * @brief   Register Addresses
     * @details Enumerated MAX77650 register addresses
     */
    typedef enum {
        INT_GLBL     = 0x00,    // Interrupt Status
        INT_CHG      = 0x01,    // Charger Interrupt Status
        STAT_CHG_A   = 0x02,    // Charger Status A
        STAT_CHG_B   = 0x03,    // Charger Status B
        ERCFLAG      = 0x04,    // Flags
        STAT_GLBL    = 0x05,    // Global Status
        INTM_GLBL    = 0x06,    // Interrupt Mask
        INT_M_CHG    = 0x07,    // Charger Interrupt Mask
        CNFG_GLBL    = 0x10,    // Global Configuration
        CID          = 0x11,    // Chip Identification Code
        CNFG_GPIO    = 0x12,    // GPIO Configuration
        CNFG_CHG_A   = 0x18,    // Charger Configuration A
        CNFG_CHG_B   = 0x19,    // Charger Configuration B
        CNFG_CHG_C   = 0x1A,    // Charger Configuration C
        CNFG_CHG_D   = 0x1B,    // Charger Configuration D
        CNFG_CHG_E   = 0x1C,    // Charger Configuration E
        CNFG_CHG_F   = 0x1D,    // Charger Configuration F
        CNFG_CHG_G   = 0x1E,    // Charger Configuration G
        CNFG_CHG_H   = 0x1F,    // Charger Configuration H
        CNFG_CHG_I   = 0x20,    // Charger Configuration I
        CNFG_SBB_TOP = 0x28,    // SIMO Buck-Boost Configuration
        CNFG_SBB0_A  = 0x29,    // SIMO Buck-Boost 0 Configuration A
        CNFG_SBB0_B  = 0x2A,    // SIMO Buck-Boost 0 Configuration B
        CNFG_SBB1_A  = 0x2B,    // SIMO Buck-Boost 1 Configuration A
        CNFG_SBB1_B  = 0x2C,    // SIMO Buck-Boost 1 Configuration B
        CNFG_SBB2_A  = 0x2D,    // SIMO Buck-Boost 2 Configuration A
        CNFG_SBB2_B  = 0x2E,    // SIMO Buck-Boost 2 Configuration B
        CNFG_LDO_A   = 0x38,    // LDO Configuration A
        CNFG_LDO_B   = 0x39,    // LDO Configuration B
        CNFG_LED0_A  = 0x40,    // LED 0 Configuration A
        CNFG_LED1_A  = 0x41,    // LED 1 Configuration A
        CNFG_LED2_A  = 0x42,    // LED 2 Configuration A
        CNFG_LED0_B  = 0x43,    // LED 0 Configuration B
        CNFG_LED1_B  = 0x44,    // LED 1 Configuration B
        CNFG_LED2_B  = 0x45,    // LED 2 Configuration B
        CNFG_LED_TOP = 0x46     // LED Configuration
    } reg_t;

    /**
     * @brief   Global Configuration settings
     * @details Enumerated global configuration settings
     */
    typedef enum {
        SBIA_LPM    = 0x20,
        SBIA_EN     = 0x10,
        NEN_PUSH    = 0x00,
        NEN_SLIDE   = 0x08,
        DBEN_100_US = 0x00,
        DBEN_30_MS  = 0x04,
        SFT_OFF     = 0x02,
        SFT_CRST    = 0x01,
    } cnfg_gbl_t;

    /**
     * @brief   Interrupt Enable Flags
     * @details Enumerated interrupt enable flags
     */
    typedef enum {
        DOD =      0x40,
        TJAL_2 =   0x20,
        TJAL_1 =   0x10,
        NEN_RISE = 0x08,
        NEN_FALL = 0x04,
        GPI_RISE = 0x02,
        GPI_FALL = 0x01,
    } intm_glbl_t;

    /**
     * MAX77650 constructor.
     *
     * @param i2c I2C object to use.
     * @param pwrHldPin Pin power hold input is connected to.
     * @param slaveAddress Slave Address of the device.
     */
    MAX77650(I2C &i2c, PinName pwrHldPin = NC, int addr = MAX77650_I2C_ADDRESS);

    /**
     * MAX77650 destructor.
     */
    ~MAX77650();

    /**
     * @brief   Read Register
     * @details Reads from the specified register
     * @param   reg The register to be read
     * @param   val Pointer for where to store the data
     * @returns 0 if no errors, -1 if error.
     */
    int readReg(reg_t reg, char *val);

    /**
     * @brief   Write Register
     * @details Writes the given value to the specified register.
     * @param   reg The register to be written
     * @param   val The data to be written
     * @returns 0 if no errors, -1 if error.
     */
    int writeReg(reg_t reg, char val);

    /**
     * @brief   Write Register
     * @details Writes the given value(s) beginning with
     * the register specified in the first buffer entry.
     * @param   buf The register and data to be written
     * @param   len The buffer length including starting register and data to be written
     * @returns 0 if no errors, -1 if error.
     */
    int writeReg(const char *buf, int len);

    /**
     * @brief   Read the CID
     * @details Read and return Chip Identification Code register value
     * @returns CID if no errors, -1 if error.
     */
    int cid(void);

    /**
     * @brief   Enable LDO
     * @details Enables LDO
     * @returns 0 if no errors, -1 if error.
     */
    int enableLDO(void);

    /**
     * @brief   Disable LDO
     * @details Disables LDO
     * @returns 0 if no errors, -1 if error.
     */
    int disableLDO(void);

    /**
     * @brief   Set SBB 2 target voltage
     * @details Select SBB 2 target output voltage.
     * @param   tv_mv The target voltage selection in milli-volts
     * @returns 0 if no errors, -1 if error.
     */
    int setSBB2VoltageMV(uint32_t tv_mv);

    /**
     * @brief   Set SBB 2 target voltage
     * @details Select SBB 2 target output voltage.
     * @param   tv_v The target voltage selection in volts
     * @returns 0 if no errors, -1 if error.
     */
    int setSBB2Voltage(float tv_v);

    /**
     * @brief   Assert Power Hold input
     * @details Asserts Power Hold input.
     * @param   pin The pin connected to Power Hold input
     * @returns 0 if no errors, -1 if error.
     */
    void assertPowerHold(void);

    /**
     * @brief   Deassert Power Hold input
     * @details Deasserts Power Hold input.
     * @param   pin The pin connected to Power Hold input
     * @returns 0 if no errors, -1 if error.
     */
    void deassertPowerHold(void);

    












    /******************************************************************************
    *                                Get register                                 *
    *******************************************************************************/
    int getRegister(reg_t reg, char dataPos);

    //******************************************************************************
    int getINT_GLBL(void);

    //******************************************************************************
    int getINT_CHG(void);

    //******************************************************************************
    int getVCHGIN_MIN_STAT(void);
    int getICHGIN_LIM_STAT(void);
    int getVSYS_MIN_STAT(void);
    int getTJ_REG_STAT(void);
    int getTHM_DTLS(void);

    //******************************************************************************
    int getCHG_DTLS(void);
    int getCHGIN_DTLS(void);
    int getCHG(void);
    int getTIME_SUS(void);

    //******************************************************************************
    int getERCFLAG(void);

    //******************************************************************************
    int getDIDM(void);
    int getLDO_DropoutDetector(void);
    int getThermalAlarm1(void);
    int getThermalAlarm2(void);
    int getDebounceStatusnEN0(void);
    int getDebounceStatusPWR_HLD(void);

    //******************************************************************************
    int getINT_M_GLBL(void);

    //******************************************************************************
    int getINT_M_CHG(void);

    //******************************************************************************
    int getBOK(void);
    int getSBIA_LPM(void);
    int getSBIA_EN(void);
    int getnEN_MODE(void);
    int getDBEN_nEN(void);
    int getSFT_RST(void);

    //******************************************************************************
    int getChipID(void);

    //******************************************************************************
    int getDBEN_GPI(void);
    int getDO(void);
    int getDRV(void);
    int getDI(void);
    int getDIR(void);

    //******************************************************************************
    int getTHM_HOT(void);
    int getTHM_WARM(void);
    int getTHM_COOL(void);
    int getTHM_COLD(void);

    //******************************************************************************
    int getVCHGIN_MIN(void);
    int getICHGIN_LIM(void);
    int getI_PQ(void);
    int getCHG_EN(void);

    //******************************************************************************
    int getCHG_PQ(void);
    int getI_TERM(void);
    int getT_TOPOFF(void);

    //******************************************************************************
    int getTJ_REG(void);
    int getVSYS_REG(void);

    //******************************************************************************
    int getCHG_CC(void);
    int getT_FAST_CHG(void);

    //******************************************************************************
    int getCHG_CC_JEITA(void);
    int getTHM_EN(void);

    //******************************************************************************
    int getCHG_CV(void);
    int getUSBS(void);

    //******************************************************************************
    int getCHG_CV_JEITA(void);

    //******************************************************************************
    int getIMON_DISCHG_SCALE(void);
    int getMUX_SEL(void);

    //******************************************************************************
    int MRT_OTP(void);
    int getSBIA_LPM_DEF(void);
    int getDBNC_nEN_DEF(void);
    int getDRV_SBB(void);

    //******************************************************************************
    int getIP_SBB0(void);
    int getTV_SBB0(void);

    //******************************************************************************
    int getADE_SBB0(void);
    int getEN_SBB0(void);

    //******************************************************************************
    int getIP_SBB1(void);
    int getTV_SBB1(void);

    //******************************************************************************
    int getADE_SBB1(void);
    int getEN_SBB1(void);

    //******************************************************************************
    int getIP_SBB2(void);
    int getTV_SBB2(void);

    //******************************************************************************
    int getADE_SBB2(void);
    int getEN_SBB2(void);

    //******************************************************************************
    int getTV_LDO(void);

    //******************************************************************************
    int getADE_LDO(void);
    int getEN_LDO(void);

    //******************************************************************************
    int getLED0_FS(void);
    int getLED0_INV(void);
    int getLED0_BRT(void);

    //******************************************************************************
    int getLED1_FS(void);
    int getLED1_INV(void);
    int getLED1_BRT(void);

    //******************************************************************************
    int getLED2_FS(void);
    int getLED2_INV(void);
    int getLED2_BRT(void);

    //******************************************************************************
    int getLED0_Period(void);
    int getLED0_Duty(void);

    //******************************************************************************
    int getLED1_Period(void);
    int getLED1_Duty(void);

    //******************************************************************************
    int getLED2_Period(void);
    int getLED2_Duty(void);

    //******************************************************************************
    int getCLK_64_S(void);
    int getEN_LED_MSTR(void);





    /******************************************************************************
    *                                Set register                                 *
    *******************************************************************************/
    int setRegister(reg_t reg, char dataPos, char value);

    //******************************************************************************
    int setINT_M_GLBL(char value);

    //******************************************************************************
    int setINT_M_CHG(char value);

    //******************************************************************************
    int setBOK(char value);
    int setSBIA_LPM(char value);
    int setSBIA_EN(char value);
    int setnEN_MODE(char value);
    int setDBEN_nEN(char value);
    int setSFT_RST(char value);

    //******************************************************************************
    int setDBEN_GPI(char value);
    int setDO(char value);
    int setDRV(char value);
    int setDI(char value);
    int setDIR(char value);

    //******************************************************************************
    int setTHM_HOT(char value);
    int setTHM_WARM(char value);
    int setTHM_COOL(char value);
    int setTHM_CLOD(char value);

    //******************************************************************************
    int setVCHGIN_MIN(char value);
    int setICHGIN_LIM(char value);
    int setI_PQ(char value);
    int setCHG_EN(char value);

    //******************************************************************************
    int setCHG_PQ(char value);
    int setI_TERM(char value);
    int setT_TOPOFF(char value);

    //******************************************************************************
    int setTJ_REG(char value);
    int setVSYS_REG(char value);

    //******************************************************************************
    int setCHG_CC(char value);
    int setT_FAST_CHG(char value);

    //******************************************************************************
    int setCHG_CC_JEITA(char value);
    int setTHM_EN(char value);

    //******************************************************************************
    int setCHG_CV(char value);
    int setUSBS(char value);

    //******************************************************************************
    int setCHG_CV_JEITA(char value);

    //******************************************************************************
    int setIMON_DISCHG_SCALE(char value);
    int setMUX_SEL(char value);

    //******************************************************************************
    int setMRT_OTP(char value);
    int setSBIA_LPM_DEF(char value);
    int setDBNC_nEN_DEF(char value);
    int setDRV_SBB(char value);

    //******************************************************************************
    int setIP_SBB0(char value);
    int setTV_SBB0(char value);

    //******************************************************************************
    int setADE_SBB0(char value);
    int setEN_SBB0(char value);

    //******************************************************************************
    int setIP_SBB1(char value);
    int setTV_SBB1(char value);

    //******************************************************************************
    int setADE_SBB1(char value);
    int setEN_SBB1(char value);

    //******************************************************************************
    int setIP_SBB2(char value);
    int setTV_SBB2(char value);

    //******************************************************************************
    int setADE_SBB2(char value);
    int setEN_SBB2(char value);

    //******************************************************************************
    int setTV_LDO(char value);

    //******************************************************************************
    int setADE_LDO(char value);
    int setEN_LDO(char value);

    //******************************************************************************
    int setLED0_FS(char value);
    int setLED0_INV(char value);
    int setLED0_BRT(char value);

    //******************************************************************************
    int setLED1_FS(char value);
    int setLED1_INV(char value);
    int setLED1_BRT(char value);

    //******************************************************************************
    int setLED2_FS(char value);
    int setLED2_INV(char value);
    int setLED2_BRT(char value);

    //******************************************************************************
    int setLED0_Period(char value);
    int setLED0_Duty(char value);

    //******************************************************************************
    int setLED1_Period(char value);
    int setLED1_Duty(char value);

    //******************************************************************************
    int setLED2_Period(char value);
    int setLED2_Duty(char value);

    //******************************************************************************
    int setLED_MSTR(char value);

private:
    I2C &i2c;
    int devAddr;
    DigitalOut *pwrHld;
};

#endif
