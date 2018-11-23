/* MAX77650.cpp
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

#include "MAX77650.h"

#define SBB2_TV_MIN_MV 800
#define SBB2_TV_MAX_MV 3950
#define SBB2_TV_LSB_MV 50

//******************************************************************************
MAX77650::MAX77650(I2C &i2c, PinName pwrHldPin, int addr) : i2c(i2c) {
    devAddr = addr;
    pwrHld = new DigitalOut(pwrHldPin, 1);
}

//******************************************************************************
MAX77650::~MAX77650() {
    delete pwrHld;
}

//******************************************************************************
int MAX77650::readReg(reg_t reg, char *val) {
    char wbuf[] = { reg };

    if (!(i2c.write(devAddr, wbuf, sizeof(wbuf), true))) {
        if (!(i2c.read(devAddr, val, 1))) {
            return MAX77650_NO_ERROR;
        }
    }

    return MAX77650_ERROR;
}

//******************************************************************************
int MAX77650::writeReg(reg_t reg, char val) {
    char wbuf[] = { reg, val };

    if (!(i2c.write(devAddr, wbuf, sizeof(wbuf)))) {
        return MAX77650_NO_ERROR;
    }

    return MAX77650_ERROR;
}

//******************************************************************************
int MAX77650::writeReg(const char *buf, int len) {
    if (!(i2c.write(devAddr, buf, len))) {
        return MAX77650_NO_ERROR;
    }

    return MAX77650_ERROR;
}

//******************************************************************************
void MAX77650::assertPowerHold(void) {
    pwrHld->write(1);
}

//******************************************************************************
void MAX77650::deassertPowerHold(void) {
    pwrHld->write(0);
}





/******************************************************************************
*                                Get register                                 *
*******************************************************************************/
int MAX77650::getRegister(reg_t reg, char dataPos) {
    char i;
    char data[1];

    if (readReg(reg, data)) {
        return MAX77650_ERROR;
    }

    *data &= dataPos;

    for(i=0; i<8; i++) {
        if(dataPos & (1 << i)) {
            break;
        }
    }

    return (*data >> i);
}

//******************************************************************************
int MAX77650::getINT_GLBL(void) {
    return getRegister(INT_GLBL, 0xFF);
}

//******************************************************************************
int MAX77650::getINT_CHG(void) {
    return getRegister(INT_CHG, 0xFF);
}

//******************************************************************************
int MAX77650::getVCHGIN_MIN_STAT(void) {
    return getRegister(STAT_CHG_A, 0x40);
}

int MAX77650::getICHGIN_LIM_STAT(void) {
    return getRegister(STAT_CHG_A, 0x20);
}

int MAX77650::getVSYS_MIN_STAT(void) {
    return getRegister(STAT_CHG_A, 0x10);
}

int MAX77650::getTJ_REG_STAT(void) {
    return getRegister(STAT_CHG_A, 0x08);
}

int MAX77650::getTHM_DTLS(void) {
    return getRegister(STAT_CHG_A, 0x07);
}

//******************************************************************************
int MAX77650::getCHG_DTLS(void) {
    return getRegister(STAT_CHG_B, 0xF0);
}

int MAX77650::getCHGIN_DTLS(void) {
    return getRegister(STAT_CHG_B, 0x0C);
}

int MAX77650::getCHG(void) {
    return getRegister(STAT_CHG_B, 0x02);
}

int MAX77650::getTIME_SUS(void) {
    return getRegister(STAT_CHG_B, 0x01);
}

//******************************************************************************
int MAX77650::getERCFLAG(void) {
    return getRegister(ERCFLAG, 0xFF);
}

//******************************************************************************
int MAX77650::getDIDM(void) {
    return getRegister(STAT_GLBL, 0xC0);
}

int MAX77650::getLDO_DropoutDetector(void) {
    return getRegister(STAT_GLBL, 0x20);
}

int MAX77650::getThermalAlarm1(void) {
    return getRegister(STAT_GLBL, 0x10);
}

int MAX77650::getThermalAlarm2(void) {
    return getRegister(STAT_GLBL, 0x08);
}

int MAX77650::getDebounceStatusnEN0(void) {
    return getRegister(STAT_GLBL, 0x04);
}

int MAX77650::getDebounceStatusPWR_HLD(void) {
    return getRegister(STAT_GLBL, 0x02);
}

//******************************************************************************
int MAX77650::getINT_M_GLBL(void) {
    return getRegister(INTM_GLBL, 0x7F);
}

//******************************************************************************
int MAX77650::getINT_M_CHG(void) {
    return getRegister(INT_M_CHG, 0x7F);
}

//******************************************************************************
int MAX77650::getBOK(void) {
    return getRegister(CNFG_GLBL, 0x40);
}

int MAX77650::getSBIA_LPM(void) {
    return getRegister(CNFG_GLBL, 0x20);
}

int MAX77650::getSBIA_EN(void) {
    return getRegister(CNFG_GLBL, 0x10);
}

int MAX77650::getnEN_MODE(void) {
    return getRegister(CNFG_GLBL, 0x08);
}

int MAX77650::getDBEN_nEN(void) {
    return getRegister(CNFG_GLBL, 0x04);
}

int MAX77650::getSFT_RST(void) {
    return getRegister(CNFG_GLBL, 0x03);
}

//******************************************************************************
int MAX77650::getChipID(void) {
    return getRegister(CID, 0x0F);
}

//******************************************************************************
int MAX77650::getDBEN_GPI(void) {
    return getRegister(CNFG_GPIO, 0x10);
}

int MAX77650::getDO(void) {
    return getRegister(CNFG_GPIO, 0x08);
}

int MAX77650::getDRV(void) {
    return getRegister(CNFG_GPIO, 0x04);
}

int MAX77650::getDI(void) {
    return getRegister(CNFG_GPIO, 0x02);
}

int MAX77650::getDIR(void) {
    return getRegister(CNFG_GPIO, 0x01);
}

//******************************************************************************
int MAX77650::getTHM_HOT(void) {
    return getRegister(CNFG_CHG_A, 0xC0);
}

int MAX77650::getTHM_WARM(void) {
    return getRegister(CNFG_CHG_A, 0x30);
}

int MAX77650::getTHM_COOL(void) {
    return getRegister(CNFG_CHG_A, 0x0C);
}

int MAX77650::getTHM_COLD(void) {
    return getRegister(CNFG_CHG_A, 0x03);
}

//******************************************************************************
int MAX77650::getVCHGIN_MIN(void) {
    return getRegister(CNFG_CHG_B, 0xE0);
}

int MAX77650::getICHGIN_LIM(void) {
    return getRegister(CNFG_CHG_B, 0x1C);
}

int MAX77650::getI_PQ(void) {
    return getRegister(CNFG_CHG_B, 0x02);
}

int MAX77650::getCHG_EN(void) {
    return getRegister(CNFG_CHG_B, 0x01);
}

//******************************************************************************
int MAX77650::getCHG_PQ(void) {
    return getRegister(CNFG_CHG_C, 0xE0);
}

int MAX77650::getI_TERM(void) {
    return getRegister(CNFG_CHG_C, 0x1C);
}

int MAX77650::getT_TOPOFF(void) {
    return getRegister(CNFG_CHG_C, 0x07);
}

//******************************************************************************
int MAX77650::getTJ_REG(void) {
    return getRegister(CNFG_CHG_D, 0xE0);
}

int MAX77650::getVSYS_REG(void) {
    return getRegister(CNFG_CHG_D, 0x1F);
}

//******************************************************************************
int MAX77650::getCHG_CC(void) {
    return getRegister(CNFG_CHG_E, 0xFC);
}

int MAX77650::getT_FAST_CHG(void) {
    return getRegister(CNFG_CHG_E, 0x03);
}

//******************************************************************************
int MAX77650::getCHG_CC_JEITA(void) {
    return getRegister(CNFG_CHG_F, 0xFC);
}

int MAX77650::getTHM_EN(void) {
    return getRegister(CNFG_CHG_F, 0x02);
}

//******************************************************************************
int MAX77650::getCHG_CV(void) {
    return getRegister(CNFG_CHG_G, 0xFC);
}

int MAX77650::getUSBS(void) {
    return getRegister(CNFG_CHG_G, 0x02);
}

//******************************************************************************
int MAX77650::getCHG_CV_JEITA(void) {
    return getRegister(CNFG_CHG_H, 0xFC);
}

//******************************************************************************
int MAX77650::getIMON_DISCHG_SCALE(void) {
    return getRegister(CNFG_CHG_I, 0xF0);
}

int MAX77650::getMUX_SEL(void) {
    return getRegister(CNFG_CHG_I, 0x0F);
}

//******************************************************************************
int MAX77650::MRT_OTP(void) {
    return getRegister(CNFG_SBB_TOP, 0x40);
}

int MAX77650::getSBIA_LPM_DEF(void) {
    return getRegister(CNFG_SBB_TOP, 0x20);
}

int MAX77650::getDBNC_nEN_DEF(void) {
    return getRegister(CNFG_SBB_TOP, 0x10);
}

int MAX77650::getDRV_SBB(void) {
    return getRegister(CNFG_SBB_TOP, 0x03);
}

//******************************************************************************
int MAX77650::getIP_SBB0(void) {
    return getRegister(CNFG_SBB0_A, 0xC0);
}

int MAX77650::getTV_SBB0(void) {
    return getRegister(CNFG_SBB0_A, 0x3F);
}

//******************************************************************************
int MAX77650::getADE_SBB0(void) {
    return getRegister(CNFG_SBB0_B, 0x08);
}

int MAX77650::getEN_SBB0(void) {
    return getRegister(CNFG_SBB0_B, 0x07);
}

//******************************************************************************
int MAX77650::getIP_SBB1(void) {
    return getRegister(CNFG_SBB1_A, 0xC0);
}

int MAX77650::getTV_SBB1(void) {
    return getRegister(CNFG_SBB1_A, 0x3F);
}

//******************************************************************************
int MAX77650::getADE_SBB1(void) {
    return getRegister(CNFG_SBB1_B, 0x08);
}

int MAX77650::getEN_SBB1(void) {
    return getRegister(CNFG_SBB1_B, 0x07);
}

//******************************************************************************
int MAX77650::getIP_SBB2(void) {
    return getRegister(CNFG_SBB2_A, 0xC0);
}

int MAX77650::getTV_SBB2(void) {
    return getRegister(CNFG_SBB2_A, 0x3F);
}

//******************************************************************************
int MAX77650::getADE_SBB2(void) {
    return getRegister(CNFG_SBB2_B, 0x08);
}

int MAX77650::getEN_SBB2(void) {
    return getRegister(CNFG_SBB2_B, 0x07);
}

//******************************************************************************
int MAX77650::getTV_LDO(void) {
    return getRegister(CNFG_LDO_A, 0x7F);
}

//******************************************************************************
int MAX77650::getADE_LDO(void) {
    return getRegister(CNFG_LDO_B, 0x08);
}

int MAX77650::getEN_LDO(void) {
    return getRegister(CNFG_LDO_B, 0x07);
}

//******************************************************************************
int MAX77650::getLED0_FS(void) {
    return getRegister(CNFG_LED0_A, 0xC0);
}

int MAX77650::getLED0_INV(void) {
    return getRegister(CNFG_LED0_A, 0x02);
}

int MAX77650::getLED0_BRT(void) {
    return getRegister(CNFG_LED0_A, 0x1F);
}

//******************************************************************************
int MAX77650::getLED1_FS(void) {
    return getRegister(CNFG_LED1_A, 0xC0);
}

int MAX77650::getLED1_INV(void) {
    return getRegister(CNFG_LED1_A, 0x02);
}

int MAX77650::getLED1_BRT(void) {
    return getRegister(CNFG_LED1_A, 0x1F);
}

//******************************************************************************
int MAX77650::getLED2_FS(void) {
    return getRegister(CNFG_LED2_A, 0xC0);
}

int MAX77650::getLED2_INV(void) {
    return getRegister(CNFG_LED2_A, 0x02);
}

int MAX77650::getLED2_BRT(void) {
    return getRegister(CNFG_LED2_A, 0x1F);
}

//******************************************************************************
int MAX77650::getLED0_Period(void) {
    return getRegister(CNFG_LED0_B, 0xF0);
}

int MAX77650::getLED0_Duty(void) {
    return getRegister(CNFG_LED0_B, 0x0F);
}

//******************************************************************************
int MAX77650::getLED1_Period(void) {
    return getRegister(CNFG_LED1_B, 0xF0);
}

int MAX77650::getLED1_Duty(void) {
    return getRegister(CNFG_LED1_B, 0x0F);
}

//******************************************************************************
int MAX77650::getLED2_Period(void) {
    return getRegister(CNFG_LED2_B, 0xF0);
}

int MAX77650::getLED2_Duty(void) {
    return getRegister(CNFG_LED2_B, 0x0F);
}

//******************************************************************************
int MAX77650::getCLK_64_S(void) {
    return getRegister(CNFG_LED_TOP, 0x02);
}

int MAX77650::getEN_LED_MSTR(void) {
    return getRegister(CNFG_LED_TOP, 0x01);
}





/******************************************************************************
*                                Set register                                 *
*******************************************************************************/
int MAX77650::setRegister(reg_t reg, char dataPos, char value) {
    char i;
    char data[1];

    if(readReg(reg, data)) {
        return MAX77650_ERROR;
    }

    *data &= (~dataPos);

    for(i=0; i<8; i++) {
        if(dataPos & (1 << i)) {
            break;
        }
    }

    return writeReg(reg, *data | ((value & (dataPos >> i)) << i));
}

//******************************************************************************
int MAX77650::setINT_M_GLBL(char value) {
    return setRegister(INTM_GLBL, 0x7F, value);
}

//******************************************************************************
int MAX77650::setINT_M_CHG(char value) {
    return setRegister(INT_M_CHG, 0x7F, value);
}

//******************************************************************************
int MAX77650::setBOK(char value) {
    return setRegister(CNFG_GLBL, 0x40, value);
}

int MAX77650::setSBIA_LPM(char value) {
    return setRegister(CNFG_GLBL, 0x20, value);
}

int MAX77650::setSBIA_EN(char value) {
    return setRegister(CNFG_GLBL, 0x10, value);
}

int MAX77650::setnEN_MODE(char value) {
    return setRegister(CNFG_GLBL, 0x08, value);
}

int MAX77650::setDBEN_nEN(char value) {
    return setRegister(CNFG_GLBL, 0x04, value);
}

int MAX77650::setSFT_RST(char value) {
    return setRegister(CNFG_GLBL, 0x03, value);
}

//******************************************************************************
int MAX77650::setDBEN_GPI(char value) {
    return setRegister(CNFG_GPIO, 0x10, value);
}

int MAX77650::setDO(char value) {
    return setRegister(CNFG_GPIO, 0x08, value);
}

int MAX77650::setDRV(char value) {
    return setRegister(CNFG_GPIO, 0x04, value);
}

int MAX77650::setDI(char value) {
    return setRegister(CNFG_GPIO, 0x02, value);
}

int MAX77650::setDIR(char value) {
    return setRegister(CNFG_GPIO, 0x01, value);
}

//******************************************************************************
int MAX77650::setTHM_HOT(char value) {
    return setRegister(CNFG_CHG_A, 0xC0, value);
}

int MAX77650::setTHM_WARM(char value) {
    return setRegister(CNFG_CHG_A, 0x30, value);
}

int MAX77650::setTHM_COOL(char value) {
    return setRegister(CNFG_CHG_A, 0x0C, value);
}

int MAX77650::setTHM_CLOD(char value) {
    return setRegister(CNFG_CHG_A, 0x03, value);
}

//******************************************************************************
int MAX77650::setVCHGIN_MIN(char value) {
    return setRegister(CNFG_CHG_B, 0xE0, value);
}

int MAX77650::setICHGIN_LIM(char value) {
    return setRegister(CNFG_CHG_B, 0x1C, value);
}

int MAX77650::setI_PQ(char value) {
    return setRegister(CNFG_CHG_B, 0x02, value);
}

int MAX77650::setCHG_EN(char value) {
    return setRegister(CNFG_CHG_B, 0x01, value);
}

//******************************************************************************
int MAX77650::setCHG_PQ(char value) {
    return setRegister(CNFG_CHG_C, 0xE0, value);
}

int MAX77650::setI_TERM(char value) {
    return setRegister(CNFG_CHG_C, 0x18, value);
}

int MAX77650::setT_TOPOFF(char value) {
    return setRegister(CNFG_CHG_C, 0x07, value);
}

//******************************************************************************
int MAX77650::setTJ_REG(char value) {
    return setRegister(CNFG_CHG_D, 0xE0, value);
}

int MAX77650::setVSYS_REG(char value) {
    return setRegister(CNFG_CHG_D, 0x1F, value);
}

//******************************************************************************
int MAX77650::setCHG_CC(char value) {
    return setRegister(CNFG_CHG_E, 0xFC, value);
}

int MAX77650::setT_FAST_CHG(char value) {
    return setRegister(CNFG_CHG_E, 0x03, value);
}

//******************************************************************************
int MAX77650::setCHG_CC_JEITA(char value) {
    return setRegister(CNFG_CHG_F, 0xFC, value);
}

int MAX77650::setTHM_EN(char value) {
    return setRegister(CNFG_CHG_F, 0x02, value);
}

//******************************************************************************
int MAX77650::setCHG_CV(char value) {
    return setRegister(CNFG_CHG_G, 0xFC, value);
}

int MAX77650::setUSBS(char value) {
    return setRegister(CNFG_CHG_G, 0x02, value);
}

//******************************************************************************
int MAX77650::setCHG_CV_JEITA(char value) {
    return setRegister(CNFG_CHG_H, 0xFC, value);
}

//******************************************************************************
int MAX77650::setIMON_DISCHG_SCALE(char value) {
    return setRegister(CNFG_CHG_I, 0xF0, value);
}

int MAX77650::setMUX_SEL(char value) {
    return setRegister(CNFG_CHG_I, 0x0F, value);
}

//******************************************************************************
int MAX77650::setMRT_OTP(char value) {
    return setRegister(CNFG_SBB_TOP, 0x40, value);
}

int MAX77650::setSBIA_LPM_DEF(char value) {
    return setRegister(CNFG_SBB_TOP, 0x20, value);
}

int MAX77650::setDBNC_nEN_DEF(char value) {
    return setRegister(CNFG_SBB_TOP, 0x10, value);
}

int MAX77650::setDRV_SBB(char value) {
    return setRegister(CNFG_SBB_TOP, 0x03, value);
}

//******************************************************************************
int MAX77650::setIP_SBB0(char value) {
    return setRegister(CNFG_SBB0_A, 0xC0, value);
}

int MAX77650::setTV_SBB0(char value) {
    return setRegister(CNFG_SBB0_A, 0x3F, value);
}

//******************************************************************************
int MAX77650::setADE_SBB0(char value) {
    return setRegister(CNFG_SBB0_B, 0x08, value);
}

int MAX77650::setEN_SBB0(char value) {
    return setRegister(CNFG_SBB0_B, 0x07, value);
}

//******************************************************************************
int MAX77650::setIP_SBB1(char value) {
    return setRegister(CNFG_SBB1_A, 0xC0, value);
}

int MAX77650::setTV_SBB1(char value) {
    return setRegister(CNFG_SBB1_A, 0x3F, value);
}

//******************************************************************************
int MAX77650::setADE_SBB1(char value) {
    return setRegister(CNFG_SBB1_B, 0x08, value);
}

int MAX77650::setEN_SBB1(char value) {
    return setRegister(CNFG_SBB1_B, 0x07, value);
}

//******************************************************************************
int MAX77650::setIP_SBB2(char value) {
    return setRegister(CNFG_SBB1_A, 0xC0, value);
}

int MAX77650::setTV_SBB2(char value) {
    return setRegister(CNFG_SBB1_A, 0x3F, value);
}

//******************************************************************************
int MAX77650::setADE_SBB2(char value) {
    return setRegister(CNFG_SBB2_B, 0x08, value);
}

int MAX77650::setEN_SBB2(char value) {
    return setRegister(CNFG_SBB2_B, 0x07, value);
}

//******************************************************************************
int MAX77650::setTV_LDO(char value) {
    return setRegister(CNFG_LDO_A, 0x7F, value);
}

//******************************************************************************
int MAX77650::setADE_LDO(char value) {
    return setRegister(CNFG_LDO_B, 0x08, value);
}

int MAX77650::setEN_LDO(char value) {
    return setRegister(CNFG_LDO_B, 0x07, value);
}

//******************************************************************************
int MAX77650::setLED0_FS(char value) {
    return setRegister(CNFG_LED0_A, 0xC0, value);
}

int MAX77650::setLED0_INV(char value) {
    return setRegister(CNFG_LED0_A, 0x20, value);
}

int MAX77650::setLED0_BRT(char value) {
    return setRegister(CNFG_LED0_A, 0x1F, value);
}

//******************************************************************************
int MAX77650::setLED1_FS(char value) {
    return setRegister(CNFG_LED1_A, 0xC0, value);
}

int MAX77650::setLED1_INV(char value) {
    return setRegister(CNFG_LED1_A, 0x20, value);
}

int MAX77650::setLED1_BRT(char value) {
    return setRegister(CNFG_LED1_A, 0x1F, value);
}

//******************************************************************************
int MAX77650::setLED2_FS(char value) {
    return setRegister(CNFG_LED2_A, 0xC0, value);
}

int MAX77650::setLED2_INV(char value) {
    return setRegister(CNFG_LED2_A, 0x20, value);
}

int MAX77650::setLED2_BRT(char value) {
    return setRegister(CNFG_LED2_A, 0x1F, value);
}

//******************************************************************************
int MAX77650::setLED0_Period(char value) {
    return setRegister(CNFG_LED0_B, 0xF0, value);
}

int MAX77650::setLED0_Duty(char value) {
    return setRegister(CNFG_LED0_B, 0x0F, value);
}

//******************************************************************************
int MAX77650::setLED1_Period(char value) {
    return setRegister(CNFG_LED1_B, 0xF0, value);
}

int MAX77650::setLED1_Duty(char value) {
    return setRegister(CNFG_LED1_B, 0x0F, value);
}

//******************************************************************************
int MAX77650::setLED2_Period(char value) {
    return setRegister(CNFG_LED2_B, 0xF0, value);
}

int MAX77650::setLED2_Duty(char value) {
    return setRegister(CNFG_LED2_B, 0x0F, value);
}

//******************************************************************************
int MAX77650::setLED_MSTR(char value) {
    return setRegister(CNFG_LED_TOP, 0x01, value);
}
