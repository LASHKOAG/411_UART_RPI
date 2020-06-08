/*
 * mbed library program
 *  INA230 High/Low-Side Measurement,Bi-Directional CURRENT/POWER MONITOR with I2C Interface
 *  by Texas Instruments
 *
 * Kevin Braun hack of INA219 code by TI
 * 23-MAR-2017
*/

#ifndef        MBED_INA230
#define        MBED_INA230

// Set data into "addr"
#define INA230_ADDR_GG              0x80
#define INA230_ADDR_GV              0x82
#define INA230_ADDR_GA              0x84
#define INA230_ADDR_GL              0x86
#define INA230_ADDR_VG              0x88
#define INA230_ADDR_VV              0x8a
#define INA230_ADDR_VA              0x8c
#define INA230_ADDR_VL              0x8e
#define INA230_ADDR_AG              0x90
#define INA230_ADDR_AV              0x92
#define INA230_ADDR_AA              0x94
#define INA230_ADDR_AL              0x96
#define INA230_ADDR_LG              0x98
#define INA230_ADDR_LV              0x9a
#define INA230_ADDR_LA              0x9c
#define INA230_ADDR_LL              0x9e

// INA230 ID
#define INA_219_DIE                 0x4000
#define INA_230_DIE                 0x2260

// INA230 register set
#define INA230_CONFIG               0x00
#define INA230_SHUNT_V              0x01
#define INA230_BUS_VOLT             0x02
#define INA230_POWER                0x03
#define INA230_CURRENT              0x04
#define INA230_CALIB                0x05
#define INA230_MASK_ENABLE          0x06
#define INA230_ALERT_LIMIT          0x07
#define INA230_DIE_ID               0xff

// CONFIG regisrer bits

//Number of Averages in 230 CONFIG register
#define INA230_AVG_1                0x0000
#define INA230_AVG_4                0x0200
#define INA230_AVG_16               0x0400
#define INA230_AVG_64               0x0600
#define INA230_AVG_128              0x0800
#define INA230_AVG_256              0x0a00
#define INA230_AVG_512              0x0c00
#define INA230_AVG_1024             0x0e00

//Bus Voltage Conv Time in 230 CONFIG REG
#define INA230_BUS_CT_140u          0x0000
#define INA230_BUS_CT_204u          0x0040
#define INA230_BUS_CT_332u          0x0080
#define INA230_BUS_CT_588u          0x00c0
#define INA230_BUS_CT_1m100         0x0100
#define INA230_BUS_CT_2m116         0x0140
#define INA230_BUS_CT_4m156         0x0180
#define INA230_BUS_CT_8m244         0x01c0

//Shunt Voltage Conv Time in 230 CONFIG REG
#define INA230_SHUNT_CT_140u        0x0000
#define INA230_SHUNT_CT_204u        0x0008
#define INA230_SHUNT_CT_332u        0x0010
#define INA230_SHUNT_CT_588u        0x0018
#define INA230_SHUNT_CT_1m100       0x0020
#define INA230_SHUNT_CT_2m116       0x0028
#define INA230_SHUNT_CT_4m156       0x0030
#define INA230_SHUNT_CT_8m244       0x0038

// Set data into "mode"
#define INA230_PAR_M_PDWN           0
#define INA230_PAR_M_SHNT_TRG       1
#define INA230_PAR_M_BUS_TRG        2
#define INA230_PAR_M_SHNTBUS_TRG    3
#define INA230_PAR_M_ADC_OFF        4
#define INA230_PAR_M_SHNT_CONT      5
#define INA230_PAR_M_BUS_CONT       6
#define INA230_PAR_M_SHNTBUS_CONT   7    // Default

// Mask/Enable and Alert/Limit regisrer bits
#define INA230_MEAL_SOL             0x8000
#define INA230_MEAL_SUL             0x4000
#define INA230_MEAL_BOL             0x2000
#define INA230_MEAL_BUL             0x1000
#define INA230_MEAL_POL             0x0800
#define INA230_MEAL_CNVR            0x0400

#define INA230_MEAL_AFF             0x0010
#define INA230_MEAL_CVRF            0x0008
#define INA230_MEAL_OVF             0x0004
#define INA230_MEAL_APOL            0x0002
#define INA230_MEAL_LEN             0x0001

// Set data into "shunt_register"
#define INA230_PAR_R_005MOHM        5
#define INA230_PAR_R_010MOHM        10
#define INA230_PAR_R_020MOHM        20
#define INA230_PAR_R_025MOHM        25
#define INA230_PAR_R_033MOHM        33
#define INA230_PAR_R_050MOHM        50
#define INA230_PAR_R_068MOHM        68
#define INA230_PAR_R_075MOHM        75
#define INA230_PAR_R_100MOHM        100


    /** 
     * Private data structure for INA230 data values.
     * 
    **/
    typedef struct {
        // I2C Address
        uint8_t addr;           /*!< I2C address*/
        //Alternate CONFIG
        uint16_t average;       /*!< CONFIG Reg - Averaging bits 11-9*/
        uint16_t bus_ct;        /*!< CONFIG Reg - Bus CT bits 8-6 */
        uint16_t shunt_ct;      /*!< CONFIG Reg - Shunt CT bits 5-3*/
        uint16_t mode;          /*!< CONFIG Reg - Mode bits 2-0*/
        // CALBLATION REG
        uint16_t cal_data;      /*!< CALIB Reg value*/
        //DIE ID REG
        uint16_t die_id_data;   /*!< Device ID  - s/b 0x2260*/
        int16_t  shunt_res;     /*!< Shunt Resistor value * 100, 100 = 0.100 ohm*/
    } INA230_TypeDef;

    /** 
     * Default values for data structure above
     * 
    **/
    const INA230_TypeDef ina230_std_paramtr = {
        // I2C Address
        INA230_ADDR_GG,
        // CONFIG Register
        INA230_AVG_16,              // averages
        INA230_BUS_CT_588u,         // bus voltage conv time
        INA230_SHUNT_CT_588u,       // bus voltage conv time
        INA230_PAR_M_SHNTBUS_CONT,  // Measure continuously both Shunt voltage and Bus voltage
        // Calibration Register
        16384,                      // Calibration data
        //Die ID Register
        0,                          // should be non-zero if read correctly
        // Shuny Resistor
        INA230_PAR_R_075MOHM        // shunt resistor value
    };

/** INA230 High/Low-Side Measurement,Bi-Directional CURRENT/POWER MONITOR with I2C Interface
 *
 * @code
 *      //
 *      // to date, only tested with...
 *          // * 0.050 shunt resistor
 *          // * I2C address 0x80 (1000000xb)
 *          // * +-0-1.5A range
 *          // * 0-2.8V range
 *      //
 * @endcode
 */
 
class INA230
{
public:
    /** Configure data pin
      * @param data SDA and SCL pins
      * @param parameter address chip (INA230_TypeDef)
      * @param or just set address or just port
      */
    INA230(PinName p_sda, PinName p_scl, const INA230_TypeDef *ina230_parameter);
    INA230(PinName p_sda, PinName p_scl, uint8_t addr);
    INA230(PinName p_sda, PinName p_scl);

    /** Configure data pin (with other devices on I2C line)
      * @param I2C previous definition
      * @param parameter address chip (INA230_TypeDef)
      * @param or just set address or just port
      */
    INA230(I2C& p_i2c, const INA230_TypeDef *ina230_parameter);
    INA230(I2C& p_i2c, uint8_t addr);
    INA230(I2C& p_i2c);

    /** Read Current data
      * @param none
      * @return current [mA]
      */
    float read_current(void);
    int16_t read_current_reg(void);
    float read_current_by_shuntvolt(void);

    /** Read Power data
      * @param none
      * @return power [w]
      */
    float read_power(void);

    /** Read Bus voltage
      * @param none
      * @return voltage [v]
      */
    float read_bus_voltage(void);

    /** Read Shunt voltage data
      * @param none
      * @return voltage [v]
      */
    float read_shunt_voltage(void);

    /** Read configration reg.
      * @param none
      * @return configrartion register value
      */
    uint16_t read_config(void);

    /** Set configration reg.
      * @param
      * @return configrartion register value
      */
    uint16_t set_config(uint16_t cfg);

    /** Read calibration reg.
      * @param none
      * @return calibration register value
      */
    uint16_t read_calb(void);

    /** Set calibration reg.
      * @param
      * @return calibration register value
      */
    uint16_t set_calb(uint16_t clb);

    /** Set I2C clock frequency
      * @param freq.
      * @return none
      */
    void frequency(int hz);

    /** Read register (general purpose)
      * @param register's address
      * @return register data
      */
    uint8_t read_reg(uint8_t addr);

    /** Write register (general purpose)
      * @param register's address
      * @param data
      * @return register data
      */
    uint8_t write_reg(uint8_t addr, uint8_t data);
/*    
    / ** Temporary display of data structure for debug
     * - Needs "Rawserial pc;" statement
     * @param none
     * @return none
     * /
    void dumpStructure();
*/    
    /** Get the Die ID value
     * @param none
     * @return register data
     */
    uint16_t read_die_id();
    
    /** Get the Mask/Enable value
     * @param none
     * @return register data
     */
    uint16_t read_mask_enable();
    
    /** Get the Alert/Limit value
     * @param none
     * @return register data
     */
    uint16_t read_alert_limit();
    
    /** Set the Mask/Enable value
     * @param data for register
     * @return data sent
     */
    uint16_t set_mask_enable(uint16_t cfg);
    
    /** Set the Alert/Limit value
     * @param data for register
     * @return data sent
     */
    uint16_t set_alert_limit(uint16_t cfg);
    
    /** Get the Shunt Resistor value
     * @param none
     * @return resistor value * 100
     */
    int16_t  get_shunt_res();

protected:
    I2C  _i2c;

    void initialize(void);

private:
    INA230_TypeDef ina230_set_data;
    int32_t scale_factor;
    uint8_t dt[4];

};

#endif  //  MBED_INA230
