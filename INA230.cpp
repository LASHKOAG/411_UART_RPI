/*
 * mbed library program
 *  INA230 High-Side Measurement,Bi-Directional CURRENT/POWER MONITOR with I2C Interface
 *  by Texas Instruments
 *
 * Copyright (c) 2015 Kenji Arai / JH1PJL
 *  http://www.page.sannet.ne.jp/kenjia/index.html
 *  http://mbed.org/users/kenjiArai/
 *      Created: January   25th, 2015
 *      Revised: May        5th, 2015
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
 * AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include    "mbed.h"
#include    "INA230.h"

INA230::INA230 (PinName p_sda, PinName p_scl, const INA230_TypeDef *ina230_parameter) :
    _i2c(p_sda, p_scl)
{
    _i2c.frequency(400000);
    ina230_set_data = *ina230_parameter;
    initialize();
}

INA230::INA230 (PinName p_sda, PinName p_scl, uint8_t addr) :
    _i2c(p_sda, p_scl)
{
    _i2c.frequency(400000);
    // Use standard setting
    ina230_set_data = ina230_std_paramtr;
    // Change user defined address
    ina230_set_data.addr = addr;
    initialize();
}

INA230::INA230 (PinName p_sda, PinName p_scl) :
    _i2c(p_sda, p_scl)
{
    _i2c.frequency(400000);
    // Use standard setting
    ina230_set_data = ina230_std_paramtr;
    initialize();
}

INA230::INA230 (I2C& p_i2c, const INA230_TypeDef *ina230_parameter) : _i2c(p_i2c)
{
    _i2c.frequency(400000);
    ina230_set_data = *ina230_parameter;
    initialize();
}

INA230::INA230 (I2C& p_i2c, uint8_t addr) : _i2c(p_i2c)
{
    _i2c.frequency(400000);
    // Use standard setting
    ina230_set_data = ina230_std_paramtr;
    // Change user defined address
    ina230_set_data.addr = addr;
    initialize();
}

INA230::INA230 (I2C& p_i2c) : _i2c(p_i2c)
{
    _i2c.frequency(400000);
    // Use standard setting
    ina230_set_data = ina230_std_paramtr;
    initialize();
}

/////////////// Read Current //////////////////////////////
//int16_t rawIreg = 0;
float INA230::read_current()
{
    dt[0] = INA230_CURRENT;
    _i2c.write((int)ina230_set_data.addr, (char *)dt, 1, true);
    _i2c.read((int)ina230_set_data.addr, (char *)dt, 2, false);
    int16_t data = (dt[0] << 8) | dt[1];
    //rawIreg = data;
    return (double)data * 0.08255;
    // 0.06605 & CalReg = 0xF000 works to 2.16A (25.9W) with 2512 0.01ohm, cal'd @ 1.500A
    // 0.07075 & CalReg = 0xE000 works to 2.31A (27.7W)
    // 0.07617 & CalReg = 0xD000 works to 2.50A (30.0W)
    // 0.08255 & CalReg = 0xC000 works to 2.70A (32.4W)
    //was
    // 0.06065 & CalReg = 0xF000 works to 1.98A (23.8W) with 1206 0.01ohm
    // 0.06500 & CalReg = 0xE000 works to 2.15A (25.9W)
    // 0.07570 & CalReg = 0xC000 works to 2.48A (29.4W)
}

/////////////// Read Current //////////////////////////////
int16_t INA230::read_current_reg()
{
    dt[0] = INA230_CURRENT;
    _i2c.write((int)ina230_set_data.addr, (char *)dt, 1, true);
    _i2c.read((int)ina230_set_data.addr, (char *)dt, 2, false);
    uint16_t data = (dt[0] << 8) | dt[1];
    return data;
}

/////////////// Read Power ////////////////////////////////
float INA230::read_power()
{
    dt[0] = INA230_POWER;
    _i2c.write((int)ina230_set_data.addr, (char *)dt, 1, true);
    _i2c.read((int)ina230_set_data.addr, (char *)dt, 2, false);
    int16_t data = (dt[0] << 8) | dt[1];
    return (float)data / 2000;
}

//---------------------------------------------------------------------
// Read Bus_volt

float INA230::read_bus_voltage()
{
    dt[0] = INA230_BUS_VOLT;
    _i2c.write((int)ina230_set_data.addr, (char *)dt, 1, true);
    _i2c.read((int)ina230_set_data.addr, (char *)dt, 2, false);
    int16_t data = (dt[0] << 8) | dt[1];
    //pc.printf("\r\nG = %+9.3f [V]   0x%x\r\n", (float)data * 0.00125f, data);
    return (float)data * 0.00125f;
}

/////////////// Read Shunt volt ///////////////////////////
float INA230::read_shunt_voltage()
{
    dt[0] = INA230_SHUNT_V;
    _i2c.write((int)ina230_set_data.addr, (char *)dt, 1, true);
    _i2c.read((int)ina230_set_data.addr, (char *)dt, 2, false);
    int16_t data = (dt[0] << 8) | dt[1];
    return (float)data;
}

float INA230::read_current_by_shuntvolt()
{
    dt[0] = INA230_SHUNT_V;
    _i2c.write((int)ina230_set_data.addr, (char *)dt, 1, true);
    _i2c.read((int)ina230_set_data.addr, (char *)dt, 2, false);
    int16_t data = (dt[0] << 8) | dt[1];
    return (float)data / 10;
//    return ((float)data / ina230_set_data.shunt_register) / 1000;
}

/////////////// Read configulation ////////////////////////
uint16_t INA230::read_config()
{
    dt[0] = INA230_CONFIG;
    _i2c.write((int)ina230_set_data.addr, (char *)dt, 1, true);
    _i2c.read((int)ina230_set_data.addr, (char *)dt, 2, false);
    uint16_t data = (dt[0] << 8) | dt[1];
    return data;
}

//---------------------------------------------------------------------
// get mask enable reg

uint16_t INA230::read_mask_enable()
{
    dt[0] = INA230_MASK_ENABLE;
    _i2c.write((int)ina230_set_data.addr, (char *)dt, 1, true);
    _i2c.read((int)ina230_set_data.addr, (char *)dt, 2, false);
    uint16_t data = (dt[0] << 8) | dt[1];
    ina230_set_data.die_id_data = data;
    return data;
}

//---------------------------------------------------------------------
// get alert limit register

uint16_t INA230::read_alert_limit()
{
    dt[0] = INA230_ALERT_LIMIT;
    _i2c.write((int)ina230_set_data.addr, (char *)dt, 1, true);
    _i2c.read((int)ina230_set_data.addr, (char *)dt, 2, false);
    uint16_t data = (dt[0] << 8) | dt[1];
    ina230_set_data.die_id_data = data;
    return data;
}

//---------------------------------------------------------------------
// get die ID

uint16_t INA230::read_die_id()
{
    dt[0] = INA230_DIE_ID;
    _i2c.write((int)ina230_set_data.addr, (char *)dt, 1, true);
    _i2c.read((int)ina230_set_data.addr, (char *)dt, 2, false);
    uint16_t data = (dt[0] << 8) | dt[1];
    ina230_set_data.die_id_data = data;
    return data;
}

//---------------------------------------------------------------------
// set mask enable register

uint16_t INA230::set_mask_enable(uint16_t cfg)
{
    uint16_t data = cfg;
    dt[0] = INA230_MASK_ENABLE;
    dt[1] = data >> 8;    // MSB 1st
    dt[2] = data & 0xff;  // LSB 2nd
    _i2c.write((int)ina230_set_data.addr, (char *)dt, 3, false);
    return data;
}

//---------------------------------------------------------------------
// set mask enable register

uint16_t INA230::set_alert_limit(uint16_t cfg)
{
    uint16_t data = cfg;
    dt[0] = INA230_ALERT_LIMIT;
    dt[1] = data >> 8;    // MSB 1st
    dt[2] = data & 0xff;  // LSB 2nd
    _i2c.write((int)ina230_set_data.addr, (char *)dt, 3, false);
    return data;
}

//---------------------------------------------------------------------
// get shunt resistor value

int16_t INA230::get_shunt_res() { return ina230_set_data.shunt_res; }

/*
//---------------------------------------------------------------------
// print stuff for degugging

extern RawSerial pc;

void INA230::dumpStructure() {
    pc.printf("Addr: %02x   A: %04x   B: %04x   S: %04x   M: %04x   Conf: %04x   Cal: %04x    Die: %04x    SR: %d\r\n", ina230_set_data.addr, ina230_set_data.average, ina230_set_data.bus_ct, 
                ina230_set_data.shunt_ct, ina230_set_data.mode,
                ina230_set_data.average | ina230_set_data.bus_ct | ina230_set_data.shunt_ct | ina230_set_data.mode,
                ina230_set_data.cal_data, ina230_set_data.die_id_data, ina230_set_data.shunt_res);
}
*/

/////////////// Set configulation /////////////////////////
uint16_t INA230::set_config(uint16_t cfg)
{
    uint16_t data = cfg;
    dt[0] = INA230_CONFIG;
    dt[1] = data >> 8;    // MSB 1st
    dt[2] = data & 0xff;  // LSB 2nd
    _i2c.write((int)ina230_set_data.addr, (char *)dt, 3, false);
    return data;
}

/////////////// Read Calibration reg. /////////////////////
uint16_t INA230::read_calb(void)
{
    dt[0] = INA230_CALIB;
    _i2c.write((int)ina230_set_data.addr, (char *)dt, 1, true);
    _i2c.read((int)ina230_set_data.addr, (char *)dt, 2, false);
    uint16_t data = (dt[0] << 8) | dt[1];
    return data;
}

/////////////// Set Calibration reg. //////////////////////
uint16_t INA230::set_calb(uint16_t clb)
{
    uint16_t data = clb;
    dt[0] = INA230_CALIB;
    dt[1] = data >> 8;    // MSB 1st
    dt[2] = data & 0xff;  // LSB 2nd
    _i2c.write((int)ina230_set_data.addr, (char *)dt, 3, false);
    return data;
}

/////////////// Read/Write specific register //////////////
uint8_t INA230::read_reg(uint8_t addr)
{
    dt[0] = addr;
    _i2c.write((int)ina230_set_data.addr, (char *)dt, 1, true);
    _i2c.read((int)ina230_set_data.addr, (char *)dt, 1, false);
    return dt[0];
}

uint8_t INA230::write_reg(uint8_t addr, uint8_t data)
{
    dt[0] = addr;
    dt[1] = data;
    _i2c.write((int)ina230_set_data.addr, (char *)dt, 2, false);
    return dt[1];
}

/////////////// Initialize ////////////////////////////////
void INA230::initialize()
{
/*
    uint16_t data = 0;
    data  = (ina230_set_data.v_max & 0x01) << 13;
    data |= (ina230_set_data.gain & 0x03) << 11;
    data |= (ina230_set_data.bus_adc_resolution & 0x0f) << 7;
    data |= (ina230_set_data.Shunt_adc_resolution & 0x0f) << 3;
    data |= (ina230_set_data.mode & 0x07);
*/
    uint16_t data = ina230_set_data.average | ina230_set_data.bus_ct | ina230_set_data.shunt_ct | ina230_set_data.bus_ct | ina230_set_data.mode;
    dt[0] = INA230_CONFIG;
    dt[1] = data >> 8;    // MSB 1st
    dt[2] = data & 0xff;  // LSB 2nd
    _i2c.write((int)ina230_set_data.addr, (char *)dt, 3, false);
    dt[0] = INA230_CALIB;
    dt[1] = ina230_set_data.cal_data >> 8;    // MSB 1st
    dt[2] = ina230_set_data.cal_data & 0xff;  // LSB 2nd
    _i2c.write((int)ina230_set_data.addr, (char *)dt, 3, false);
    scale_factor = 0;
}

/////////////// I2C Freq. /////////////////////////////////
void INA230::frequency(int hz)
{
    _i2c.frequency(hz);
}



