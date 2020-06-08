/* 
 */

#include "mbed.h"
#include "platform/mbed_thread.h"
#include "stm32_standby.h"
#include "INA230.h"
// Blinking rate in milliseconds
#define BLINKING_RATE_MS 500

DigitalOut pinHIGH(PC_13);
DigitalOut pinLED(PB_3);

DigitalOut led2(PB_12);

#define FLAG_INTERRUPT_EVENT (1U<<0)
#define FLAG_INTERRUPT_EVENT_SLEEP (1U<<1)

bool flag_get_UART=true;

typedef struct 
{
	uint32_t command;
	uint32_t length;
	int8_t* buff;
}tcp_packet_t;

typedef struct
{
    uint32_t hz;
    float current_mA;
    float power_W;
    float bus_voltage_V;
    float shunt_voltage_V;
    int16_t current_reg;
    uint16_t config;
    uint16_t calib;
    uint16_t die_id;
    uint8_t addr;
    uint8_t rsrvd_byte;
}INA_230;

#define TCP_EC_SUCCESS        0
#define TCP_EC_UNSUCCESS     -1

#define CMD_ANSWER						1
#define CMD_GET_INAINFO				2
#define CMD_SHUTDOWN					3

DigitalIn       btn(USER_BUTTON);
DigitalOut      led(LED1);
RawSerial       pc(SERIAL_TX, SERIAL_RX,9600);
volatile char   c = '\0'; // Initialized to the NULL character

bool flag = false;
#define FLAG_TEST (1U << 2)
EventFlags event_flags;
char MSV[100]={10, };
char MSVsend[100]={10, };

const uint8_t sizeBuffer = 100;
int8_t Buffer[sizeBuffer];

const uint8_t sizeTempMSV = 200;
int8_t TempMSV[sizeTempMSV];
//char MSV[10]=" ";
//int counters = 0;
char TempBufferSend[50]={0, };
int8_t counters = 0;
bool interrupted = false;
//----------------------------------------------------------------------------
//https://os.mbed.com/users/mbed_official/code/mbed-src/file/410346174f7a/targets/cmsis/TARGET_STM/TARGET_NUCLEO_F103RB/system_stm32f10x.c/
// #define SYSCLK_FREQ_24MHz  24000000
// uint32_t SystemCoreClock = SYSCLK_FREQ_24MHz;        /*!< System Clock Frequency (Core Clock) */
// static void SetSysClockTo24(void);
// SetSysClockTo24();
//---------------------------------------------------------------------------

//------------------------------------
// Hyperterminal configuration
// 9600 bauds, 8-bit data, no parity
//------------------------------------
// Serial pc(SERIAL_TX, SERIAL_RX,9600);

// EventFlags eventFlags;

Thread testThread;
Thread sleepThread;
// InterruptIn btnInterrupt(PA_0);
InterruptIn btnInterrupt(USER_BUTTON);
Timeout btnInterrupt_timeout;       // Used for debouncing
// Serial device(USBTX, USBRX);  // tx, rx
//I2C i2c(I2C_SDA , I2C_SCL );  //I2C i2c(p9, p10);        // sda, scl
 
//const int addr = 0x00 ... 0x07; // define the correct I2C Address   

bool btnPressed = false;
float current_mA = 0.0f;
int16_t current_reg = 0;
uint32_t hz = 0;
uint8_t addr = 0;
float power_W = 0.0f;
float bus_voltage_V = 0.0f;
float shunt_voltage_V = 0.0f;
uint16_t config=0;
uint16_t calib=0;
uint16_t die_id=0;

void call_BtnPressed()
{
  event_flags.set(FLAG_INTERRUPT_EVENT);  
}

void call_checkBtnPressed()
{
    if(btnInterrupt.read()==0){btnPressed = true;}
}

// void call_testThread(){
//   int n=4;
//   while(1){
//     event_flags.wait_all(FLAG_INTERRUPT_EVENT);
//     // led3= 1;
//       while (n--) {
//         led3 = !led3;
//         thread_sleep_for(BLINKING_RATE_MS);
//       }
//     event_flags.clear(FLAG_INTERRUPT_EVENT);
//     n=4;
//   }
// }

void call_sleepThread(){
  while(1){
    event_flags.wait_all(FLAG_INTERRUPT_EVENT_SLEEP);
    sleep();
    event_flags.clear(FLAG_INTERRUPT_EVENT_SLEEP);
  }
}


void rx_interrupt() {
    char ch;
    if(pc.readable()){  //Determine if there is a character available to read
        ch = pc.getc(); //Read char from terminal
            //int charIndex = (ch - '0') - 49;  //Conversion from char to int where letter 'a' is 0;
        MSV[counters++] = ch;
    }
    event_flags.set(FLAG_TEST);
    return;
}


int8_t send_message (tcp_packet_t* ans_struct, RawSerial* _pc){
	int32_t len_send=sizeof(ans_struct->command)+sizeof(ans_struct->length) + ans_struct->length;
    // pc.putc(len_send);  //для отладки
	memset(&TempMSV[0], 0xFF, 200);
    uint8_t pos=0;
		memcpy(&TempMSV[pos], &ans_struct->command, sizeof(ans_struct->command));
			pos+=sizeof(ans_struct->command);
		memcpy(&TempMSV[pos], &ans_struct->length, sizeof(ans_struct->length));
			pos+=sizeof(ans_struct->length);
		memcpy(&TempMSV[pos], &ans_struct->buff[0], ans_struct->length);
    
    for(uint8_t i=0; i <len_send; ++i){     //send message
        _pc->putc(TempMSV[i]);
    }
	return 0;
}

void get_data_ina(){
    INA_230 ina;
    memset(&ina, 0xFF, sizeof(INA_230));
    //================================================================================
    ina.hz = 10;                    //для отладки
    ina.current_mA = 10.7;          //для отладки
    ina.power_W = 11.8;             
    ina.bus_voltage_V = 12.9;
    ina.shunt_voltage_V = 13.10;
    ina.current_reg = 14;
    ina.config = 15;
    ina.calib = 16;
    ina.die_id = 17;
    ina.addr = 18;
            //заслать ответ
    uint32_t res = TCP_EC_SUCCESS;
    tcp_packet_t ans;
    uint8_t pos = 0;
    uint32_t packet_command = 2; 
    memset(&ans, 0x00, sizeof(tcp_packet_t));
    memset(&Buffer[0], 0xFF, sizeBuffer );
    ans.command = CMD_ANSWER;
    ans.length = sizeof(ans.command)+sizeof(res)
                +sizeof(ina.hz)+sizeof(ina.current_mA)
                +sizeof(ina.power_W)+sizeof(ina.bus_voltage_V)+sizeof(ina.shunt_voltage_V)
                +sizeof(ina.current_reg)+sizeof(ina.config)+sizeof(ina.calib)
                +sizeof(ina.die_id)+sizeof(ina.addr);  //+всё в пакете
    // ans.length = sizeof(uint32_t)+sizeof(uint8_t)                        //sizeof(ans.command)+sizeof(res)
    //             +sizeof(uint32_t)+sizeof(float)                         //sizeof(ina.hz)+sizeof(ina.current_mA)
    //             +sizeof(float)+sizeof(float)+sizeof(float)              //sizeof(ina.power_W)+sizeof(ina.bus_voltage_V)+sizeof(ina.shunt_voltage_V)
    //             +sizeof(int16_t)+sizeof(uint16_t)+sizeof(uint16_t)      //sizeof(ina.current_reg)+sizeof(ina.config)+sizeof(ina.calib)
    //             +sizeof(uint16_t)+sizeof(uint8_t);  //+всё в пакете     //sizeof(ina.die_id)+sizeof(ina.addr);
    // ans.length = sizeof(ans.command)+sizeof(res) + sizeof(INA_230);  //+всё в пакете 

    // pc.putc(ans.length);  //для отладки
    // memcpy(&Buffer[pos], (char*)&packet.command, sizeof(packet.command));
    memcpy(&Buffer[pos], (char*)&packet_command, sizeof(packet_command));
        // pos+= sizeof(packet.command);
        pos+= sizeof(packet_command);
    memcpy(&Buffer[pos], (char*)&res, sizeof(res));
        pos+=sizeof(res);
            //+всё в пакете
    memcpy(&Buffer[pos], (char*)&ina.hz, sizeof(ina.hz));
        pos+=sizeof(ina.hz);
    memcpy(&Buffer[pos], (char*)&ina.current_mA, sizeof(ina.current_mA));
        pos+=sizeof(ina.current_mA);
    memcpy(&Buffer[pos], (char*)&ina.power_W, sizeof(ina.power_W));
        pos+=sizeof(ina.power_W);
    memcpy(&Buffer[pos], (char*)&ina.bus_voltage_V, sizeof(ina.bus_voltage_V));
        pos+=sizeof(ina.bus_voltage_V);
    memcpy(&Buffer[pos], (char*)&ina.shunt_voltage_V, sizeof(ina.shunt_voltage_V));
        pos+=sizeof(ina.shunt_voltage_V);
    memcpy(&Buffer[pos], (char*)&ina.current_reg, sizeof(ina.current_reg));
        pos+=sizeof(ina.current_reg);
    memcpy(&Buffer[pos], (char*)&ina.config, sizeof(ina.config));
        pos+=sizeof(ina.config);
    memcpy(&Buffer[pos], (char*)&ina.calib, sizeof(ina.calib));
        pos+=sizeof(ina.calib);
    memcpy(&Buffer[pos], (char*)&ina.die_id, sizeof(ina.die_id));
        pos+=sizeof(ina.die_id);
    memcpy(&Buffer[pos], (char*)&ina.addr, sizeof(ina.addr));
        pos+=sizeof(ina.addr);

    pos=0;
    ans.buff = &Buffer[0];
    send_message(&ans, &pc);

    memset(&Buffer[0], 0xFF, sizeBuffer);
}


int main()
{
  //hal_deepsleep();
  //https://os.mbed.com/questions/79328/Wake-Up-from-sleep-mode-with-RTC-interru/

    for(int8_t i=0; i<6; ++i){
        led = !led;
        wait(0.2); 
    }
    pc.format(8, SerialBase::None, 1);
        // pc.attach(callback(rx_interrupt), Serial::RxIrq);
    pc.attach(&rx_interrupt, Serial::RxIrq);
    // pc.attach(&onCharReceived);

    //i2c.установить_частоту
 
    char regaddr[1];
    char readdata[8]; // room for length and 7 databytes
    char writedata[9]; // room for reg address, length and 7 databytes


    pinHIGH = 0;
    pinLED = 0;
    led2 = 0;

    // if(btn.read() == 0){
    //   for(int8_t i=0; i<6; ++i){
    //     led = !led;
    //     wait(1); 
    //   }
    // }
   
    btnInterrupt.mode(PullUp);
    btnInterrupt.fall(&call_BtnPressed);

    while(1){
        event_flags.wait_all(FLAG_INTERRUPT_EVENT);  //прерывание произошло,
        // led=1;                                            //необходимо проверить состояние кнопки на удержание
        btnInterrupt.disable_irq ();                //меняем режим работы кнопки (прерывание отключено)
        // if(btnInterrupt.read()==0){
        //     btnInterrupt_timeout.attach(callback(call_checkBtnPressed), 0.5); // Debounce time 300 ms
        // }
        
        if(btnInterrupt.read()==0){
          ThisThread::sleep_for(1900);
        }
        if(btnInterrupt.read()==0){btnPressed=true;}

        if(btnPressed){
          led=1;
          btnPressed = false;
          pinHIGH = 1;                            //пин подачи питания от акб
          pinLED = 1;                             //LED сигнализирующий, что питание подано
          flag_get_UART = true;
          led2=1;
          led=1;

          while(flag_get_UART){                     //идёт общение по UART
            event_flags.wait_all(FLAG_TEST);        //ждем прерывания по UART
            event_flags.clear(FLAG_TEST);
            ThisThread::sleep_for(10);              //необходим таймаут

            char action=MSV[0];                     //данный буффер собирается в обработчике прерываний

            switch (action)
            {
            case 0x02:                              //команда получения данных с INA
                for(int8_t i=0; i<16; ++i){
                    led = !led;
                    wait(0.2); 
                }
                get_data_ina();
                break;
            case 0x03:                              //команда выключения
                for(int8_t i=0; i<16; ++i){
                    led = !led;
                    // wait(1); 
                }
                flag_get_UART = false;
                btnPressed = false;
                pinHIGH = 0;
                pinLED = 0;
                led2 = 0;
                led=0;
                btnInterrupt.enable_irq ();
                break;
            
            default:
                break;
            }

            memset(&MSV[0], 0xFF, 100 );
            counters=0;
            //-----------------------------------------------------------------

            event_flags.clear(FLAG_TEST);
          }
                    //работаем с INA-----------------------------------------------
            time_t seconds = time(NULL);
                    // read the data
            // INA230 *ina230 =new  INA230(i2c);
            // delete ina230;
                        
            // // void frequency(int hz);
            // current_mA = read_current();
            // current_reg = read_current_reg();
            // // float read_current_by_shuntvolt(void);
            // power_W = read_power();
            // bus_voltage_V = read_bus_voltage();
            // shunt_voltage_V = read_shunt_voltage();
            // config = read_config();  
            // // uint16_t set_config(uint16_t cfg);
            // calib = read_calb(void);
            // die_id = read_die_id();
            // uint16_t set_calb(uint16_t clb);
            // uint8_t write_reg(uint8_t addr, uint8_t data);
            // uint8_t read_reg(uint8_t addr);

            // uint16_t read_mask_enable();

            // uint16_t read_alert_limit();

            // uint16_t set_mask_enable(uint16_t cfg);

            // uint16_t set_alert_limit(uint16_t cfg);

            // int16_t  get_shunt_res();
            //--------------------------------------------------------------------------
            //==========================================================================
            //собрать сообщение для отправки
            //==========================================================================
            //UART
            //pc.printf("This program runs since %d seconds.\n", i++);

            // regaddr[0] = 0xE0;
            // i2c.write(addr, regaddr, 1, true);  // select the register, no I2C Stop
            // i2c.read(addr, readdata, 8);        // read the length byte and the 7 databytes
            // wait (1);
            //         // print the data to the screen
            //     printf("Register 0x%x = 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\r\n",
            //             regaddr[0],
            //             readdata[1], readdata[2], readdata[3], readdata[4],
            //             readdata[5], readdata[6], readdata[7] );
            // wait(1);
            //         // copy the data, starting with register address
            // writedata[0] = regaddr[0];  // register address
            // writedata[1] = readdata[0]; // length, should be 7
            // writedata[2] = readdata[1]; // byte 1
            // writedata[3] = readdata[2];
            // writedata[4] = readdata[3];
            // writedata[5] = readdata[4];
            // writedata[6] = readdata[5];
            // writedata[7] = readdata[6];
            // writedata[8] = readdata[7]; // byte 7
            //     // write the data
            // i2c.write(addr, writedata, 9); // select the register, 
            //                                 // write the length, write 7 databytes      
            // wait(1);
            // while (1){

    
            // }
        }else{
            btnInterrupt.enable_irq ();
            btnPressed = false;
            pinHIGH = 0;
            pinLED = 0;
        }

          //фиксируем время
        // if(testFlag){
        //     led3=1;
        // }else{
            // for(int i=0; i<6; ++i) {

                
                // thread_sleep_for(BLINKING_RATE_MS);
            // }
        // }
        // ThisThread::sleep_for(10000);
    }

    
//    for(int i=0; i<6; ++i) {
//        led2 = !led2;
//        thread_sleep_for(BLINKING_RATE_MS);
//    }
    // time_t seconds_1 = time(NULL);
    // time_t seconds_2 = time(NULL);

// led2=0;
    

    

    // 
    // ThisThread::sleep_for(1000);
    // if(btnInterrupt.read()==1){
    //         //процесс в сон
    //     testFlag=false;
    //     if(testFlag){ btnInterrupt.enable_irq(); }  //включили прерывания
    //     //сон
    // }  //удержание более секунды, выкл МК (сон)
    // else{ //старт программы
    //     led3=1; //подача основного питания прибора
    //     //led4=1; //светодиод, что работает прибор
    //     //взять дата время
    //     //опрос INA
    //     //посылаем в UART сообщение
    //     //получаем из UART сообщение и разбираем ответ
    //     //switch-case (ответ):
    //     //(ответ):            //процесс в сон
    //     // testFlag=false;
    //     // if(testFlag){ btnInterrupt.enable_irq(); }  //включили прерывания
    //     //сон

    // }


        // this is required!
    // set_time(0);
// led3=0;
    // printf("Hello world, going to sleep for 5 seconds\n");
                // if(testFlag==true){ testFlag=false; btnInterrupt.enable_irq(); }
                // led2=0;
    // standby(5);
    // sleep();
// system_reset();
    // after 5 seconds the application will run from the beginning again
    

//    while (true) {
//        led2 = !led2;
//        thread_sleep_for(BLINKING_RATE_MS);
//    }

}


//RawSerial use  pc.putc('b')
//Serial use pc.printf("b")