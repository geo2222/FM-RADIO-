/*********************************12/10/2018**************************
FM radio SI4703 based
piloted via BT and Smartphone AIM App
for radio I got inspired by the code from sparkfun (2011) by Nathan Seidle as there is no library and so that I can optimize the memory space by keeping just what needed
for RDS I used the GPIO2 => interupt low level for 5 ms when a RDS group text (4 bytes )is available 
Display will be done via 4X20 LCD I2C screen
commands used via blue tooth with android App:
  v+  => increase volume > prog answer with vx(x=volume)
  v-     decrease volume > prog answer with vx(x=volume)
  f+     increase frequence > prog answer with fx(x=frequency) 
  f-     decrease frequence > prog answer with fx(x=frequency)
  su     seek up > prog answer with fx(x=frequency)
  sd     seek down > prog answer with fx(x=frequency)
  sav    save the current frequency in memory
  prefd  select station before current station
  prefu  select station following in memory
  reset  clear preselected radio memory (set pointer to zero)
  Hello  when BT connects App sends Hello for receiving update of current data running in the radio (volume current channel,preselected radio number if even 
  bye    deconnection 
  pow    power supply mode
  bat    battery supply mode
  lb     low battery level
 */
#include <EEPROM.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>
#define scroll_speed_RDS 500 //scrolling speed for RDS display
//*******************************************************************************
#define FAIL  0
#define SUCCESS  1
#define SI4703 0x10 // I2C address of Si4703 
#define I2C_FAIL_MAX  10 //This is the number of attempts we will try to contact the device before erroring out
#define SEEK_DOWN  0 //Direction used for seeking. Default is down
#define SEEK_UP  1
//*********************Define the register names**************************************
// frequency dat's for Europe
#define offset_frequency 875 // !!!!gotoChannel requires the frequency + offset depending on the country=offset_frequency for europe
#define max_freq 1080
#define min_freq 875
//***************************
#define DEVICEID 0x00
#define CHIPID  0x01
#define POWERCFG  0x02
#define CHANNEL  0x03
#define SYSCONFIG1  0x04
#define SYSCONFIG2  0x05
#define STATUSRSSI  0x0A
#define READCHAN  0x0B
#define RDSA  0x0C
#define RDSB  0x0D
#define RDSC  0x0E
#define RDSD  0x0F
//**************************Register 0x02 - POWERCFG**********************************
#define SMUTE  15
#define DMUTE  14
#define SKMODE  10
#define SEEKUP  9
#define SEEK  8
//**************************Register 0x03 - CHANNEL***********************************
#define TUNE  15
//**************************Register 0x04 - SYSCONFIG1********************************
#define RDS  12
#define RDSIEN 15 //enable RDS interrup
#define DE  11
//**************************Register 0x05 - SYSCONFIG2********************************
#define SPACE1  5
#define SPACE0  4
//**************************Register 0x0A - STATUSRSSI********************************
#define RDSR  0x8000
#define STC  14
#define SFBL  13
#define AFCRL  12
#define RDSS  11
#define STEREO  8
//*****************************miscallenous*************************************
#define debounce_time 20 //20 ms for debouncing switches
#define tempo_task 5000  //tempo for periodic tasks in ms
#define tempo_flash 1000 //flash when battery low
#define lcd_addr 0x27
#define addr_vol 0// to save the current radio data (Volume)and first address for storage of data's
#define addr_freq 1//frequency
#define pref_nxtindex 2// nd preselected radio
#define first_pref 3// address of first preselected radio
# define max_BT_attempts 2 // max number of off attempts to receive ack from android app
//*********************************************I2C Bus**************************
int resetPin = 2;
int SDIO = A4; //SDA/A4 on Arduino
int SCLK = A5; //SCL/A5 on Arduino
//********************************************switches***************************
/*switches for volume,frequency,seek,preselected radios use Analog input A0 */
boolean flg_button=false;
int switch_pin=A0;
int val_but;
int val;
long unsigned tempo_debounce; //deboucing temp for switches
//********************************************led display************************
int power_pin=8;//main power supply indicator (green led)
int battery_pin=7;//battery mode supply (orange led)
int BT_pin=6;//BT connected (blue led)
int power_mes=A2;//mesure the voltage to determine weither the power supply is on or battery
boolean bat_low=false;
boolean led_bat_state=false;
long unsigned bat_flash;
//********************************************Radio*****************************
long unsigned temp_tune;
char printBuffer[10];
uint16_t si4703_registers[16]; //There are 16 registers, each 16 bits large
byte c;
String radio_mem;
String prev_mem;
byte rssi;
byte prev_rssi;
boolean flg_disp=true;/* as the channel is displayed in readChannel() ,I use this flag when 
                      I don't want to send the frequency to LCD and smartphone app*/
boolean flg_stereo;
boolean prev_flg;
boolean flg_force=false;//enforce the display when making the radio status(cycling tasks or answer to android app)
byte current_vol=0;
int i;
int j;
int k;// address offset in EEprom
int pref_index;// index of next preselected radio to save
int current_index;// current preselected radio  (99 if not a preselected radio)
//**********************************LCD display*********************************
long unsigned temp_tasks;
int row;
int col;
int lcd_col=0; // column index for scrolling RDS text
LiquidCrystal_I2C lcd(lcd_addr, 20, 4);
//**********************************BT messages exchanges************************
boolean flg_BT_connected=false;
boolean flg_BT_ack=false;
boolean flg_BT_send=true;/*used for the control loop of BT exchanges 
for each message sent by the prog to android App we wait for "ok" response before sending new data*/
String txt="";
int count_BT_error=0;// if more than 2 missing ack from android app bluetooth is declared disconnected
long unsigned temp_control_loop;
int currentChannel;
//***********************************RDS parser**********************************
int gtype;
int text_pointer;
char text_buffer[64];
boolean new_txt=false;// flag to store RSD blocks
boolean flg_print_RDS=false;
boolean flg_RDS=false;// GPIO2 interrupt
long unsigned scroll_temp;// tempo for RDS LCD scrolling
String global_RDS="";
String prev_RDS="";
char text_rds[65];
int mess_index;
int prev_index;
int block_count=0;
int space_count=0;
int nb_char; // nb char of RDS text depending on group type( 64 for gtype=4, 32 for gtype=5)
//**************************LCD*********************************************************
SoftwareSerial BT(4,5);
//**************************data save*************************************************
/* need to store data in EEprom
 *  0= current volume
 *  1=current frequency
 *  2=index max of preselected radio
 *  3 to 23 frequencies of preselected radio 
 */
//**********************************************************************************
void setup() 
{                
  txt.reserve(68);
  global_RDS.reserve (65);
  prev_RDS.reserve (65);
  Serial.begin(38400);
  /**********************to be run once to set up the baud rate of HC-06**************************
   * Serial.begin(9600);//factory default speed                                                  *
   * BT.print("AT+BAUD5");// for software version 2                                              * 
   * BT.print("AT+UART=19200,0,0");//as the AT commands have entirely changed with version 3.0!! *
   ***********************************************************************************************/
  BT.begin(19200);
  pinMode(3,INPUT_PULLUP);
  attachInterrupt (digitalPinToInterrupt(3),get_RDS,FALLING);
  //************************Init Radio*******************************************
  pinMode(resetPin, OUTPUT);
  pinMode(SDIO, OUTPUT); //SDIO is connected to A4 for I2C
  digitalWrite(SDIO, LOW); //A low SDIO indicates a 2-wire interface
  digitalWrite(resetPin, LOW); //Put Si4703 into reset
  delay(2); //Some delays while we allow pins to settle
  digitalWrite(resetPin, HIGH); //Bring Si4703 out of reset with SDIO set to low and SEN pulled high with on-board resistor
  // ********************raz led's**************
  pinMode(power_pin, OUTPUT);
  pinMode(battery_pin, OUTPUT);
  pinMode(BT_pin, OUTPUT);
  digitalWrite(power_pin, LOW);
  digitalWrite(battery_pin, LOW);
  digitalWrite(BT_pin, LOW);
  delay(10); //Allow Si4703 to come out of reset
  // and start wire with LCD********************
  Wire.begin(); //Now that the unit is reset and I2C inteface mode, we need to begin I2C
  si4703_readRegisters(); //Read the current register set
  si4703_registers[0x07] = 0x8100; //Enable the oscillator, from AN230 page 9, rev 0.61 (works)
  si4703_updateRegisters(); //Update
  delay(500); //Wait for clock to settle - from AN230 page 9
  si4703_readRegisters(); //Read the current register set
  si4703_registers[POWERCFG] = 0x4001; //Enable the IC
  si4703_registers[SYSCONFIG1] |= (1<<RDS); //Enable RDS
  si4703_registers[SYSCONFIG1] |= (1<<DE); //50kHz Europe setup
  si4703_registers[SYSCONFIG1] |= (1<<RDSIEN);// enable RDS interrupt
  si4703_registers[SYSCONFIG1] &=0xFFF3; //reset GPIO2 [1:0] 
  si4703_registers[SYSCONFIG1] |=0x0004; //set GPIO2 [1:0] to 01 => GPIO2 rds interrupt
  si4703_registers[SYSCONFIG2] |= (1<<SPACE0); //100kHz channel spacing for Europe
  si4703_registers[SYSCONFIG2] &= 0xFFF0; //Clear volume bits
  si4703_registers[SYSCONFIG2] |= current_vol; //Set volume to saved volume
  si4703_updateRegisters(); //Update
  delay(200); //Max powerup time, from datasheet page 13
  mes_power();
  //*****************************************************************************
  lcd.begin();
  lcd.backlight();
  //***********************************LCD scrren init***************************
  row=2;
  col=0;
  txt="Mem:";
  lcd_display();
  row=0;
  col=0;
  txt="Freq:";
  lcd_display();
  row=0;
  col=12;
  txt="lev:";
  lcd_display();  
  row=1;
  col=0;
  txt="Volume:";
  lcd_display();
  row=2;
  col=8;
  txt="Mode:";
  lcd_display();
  // *******************************get saved data's after poweron*******************
  k=addr_freq;
  get_num();
  currentChannel=i+offset_frequency;//saved channel
  k=pref_nxtindex;
  get_num();
  pref_index=i;
  // get channel information returns stereo flag,rds flag, rssi value and preselected channel index if even
  gotoChannel(currentChannel);
  radio_status();
  i=readChannel();// to display and send to androisd app the current channel
  // display the current preselected channel number if even or "?" if no
  row=2;
  col=4;
  if (current_index==99)//number of preselected radio station
  {
    txt="?";
  }
  else
  {
    txt=String(current_index);
  }
  lcd_display();
  delay(1000);
  k=addr_vol;
  get_num();
  current_vol=i;
  row=1;
  col=7;  
  txt=String(current_vol);
  lcd_display();  
  set_vol();
}
void loop() 
{
  //*******************************************flash bat led when battery level is low****************
  if (bat_low && millis()-bat_flash>tempo_flash)
  {
    bat_flash=millis();
    led_bat_state=!led_bat_state;
    digitalWrite(battery_pin,led_bat_state);  
  }
  //**********************************************RDS tasks********************************
  if (flg_RDS)// GPIO2 interrupt on pin D3
  {
    get_RT();// get Radio text
  }
  if (flg_print_RDS)// the RDS text is different than the previous one => display it
  {
    txt="r";
    txt+=global_RDS;
    BT_send();
    flg_print_RDS=false;
  }
  if (millis()-scroll_temp>scroll_speed_RDS)//scrooling of RDS LCD display
  {
    RDS_scroll();  
  }
  //**************************************periodic tasks******************************
  if(millis()-temp_tasks>tempo_task)
  {
    mes_power();
    radio_status();// when performing radio_status only the parameters which have changed are sent unless flg_force is set   
  }
//****************************************switches tasks*******************************
//polling the switches
  val_but=analogRead(switch_pin);
  if (val_but>1000 && millis()-tempo_debounce>300 && flg_button)
  {
    flg_button=false;//when state button changes stop scanning button for 300ms
  }
  if(val_but<1000 && !flg_button)// a switch is pushed
  {
    tempo_debounce=millis();
    flg_button=true;
    while (millis()-tempo_debounce<20){}//debouncing value confirmed
    txt="";
    val_but=analogRead(switch_pin)-5;// minus 5 due to resistors tolerance
    k=9-((val_but*6800)/(1023-val_but))/680;//use voltage divider with 680 ohm resistors and one 6,8Kohm
    switch(k)
    {
      case 9://v+
        txt="v+";
        break;
      case 8://f+
        txt="f+";
        break;
      case 7://seek+
        txt="su";
        break;
      case 6://prefu
        txt="prefu";
        break;
      case 5://save
        txt="sav";
        break;
      case 0://v-
        txt="v-";
        break;
      case 1://f-
        txt="f-";
        break;
      case 2://seek-
        txt="sd";
        break;
      case 3://prefd
        txt="prefd";
        break;
      case 4://reset
        txt="reset";
        break;
      default: 
        break;
    }
    decode_txt();
  } 
// scanning for BT messages from Android App
  if (BT.available())
  {
    BT_receive();
    //txt command for radio from Android App
    decode_txt();
  }
}
/*tune to a given channel
SI 4703 needs a value calculated from the min freqency value and given in 100 000khz
example : a frequency of 97.3  will be 98 for the SI4703
*/
void gotoChannel(int newChannel)
{
  newChannel *= 10; //973 * 10 = 9730
  newChannel -= min_freq*10; //9730 - 8750 = 980
  newChannel /= 10; //980 / 10 = 98
  //These steps come from AN230 page 20 rev 0.5
  si4703_readRegisters();
  si4703_registers[CHANNEL] &= 0xFE00; //Clear out the channel bits
  si4703_registers[CHANNEL] |= newChannel; //Mask in the new channel
  si4703_registers[CHANNEL] |= (1<<TUNE); //Set the TUNE bit to start
  si4703_updateRegisters();
  si4703_readRegisters();
  //delay(1000);
  //Poll to see if STC is set
  temp_tune=millis();
  while(1 || millis()-temp_tune>10000) 
  {
    si4703_readRegisters(); 
    if( (si4703_registers[STATUSRSSI] & (1<<STC)) != 0) 
    {
      break; //Tuning complete!
    }

  }
  si4703_readRegisters();
  si4703_registers[CHANNEL] &= ~(1<<TUNE); //Clear the tune after a tune has completed
  si4703_updateRegisters();
  //Wait for the si4703 to clear the STC as well
  while(1) 
  {
    si4703_readRegisters();
    if( (si4703_registers[STATUSRSSI] & (1<<STC)) == 0) 
    {
      break; //Tuning complete!
    }
  }
}

//Reads the current channel from READCHAN
//Returns a number like 973 for 97.3MHz so increased with offset_frequency
int readChannel(void) 
{
  si4703_readRegisters();
  i = si4703_registers[READCHAN] & 0x03FF; //Mask out everything but the lower 10 bits    
  k=addr_freq;//Addr of frequency in eeprom
  put_num();
  i +=offset_frequency;
  if (flg_disp)
  {
    sprintf(printBuffer, "%02d.%01d", i / 10, i % 10);
    row=0;
    col=0;
    txt="          ";
    lcd_display();
    txt="Freq:";
    txt+=printBuffer;
    lcd_display();
    txt="f";
    txt+=printBuffer;
    BT_send(); 
  }
  return(i);
}
//Seeks out the next available station
//Returns the freq if it made it
//Returns zero if failed
byte seek(byte seekDirection)
{
  si4703_readRegisters();
  //Set seek mode wrap bit
  si4703_registers[POWERCFG] |= (1<<SKMODE);//Disallow wrap - Seek will stay in the frequency range
  if(seekDirection == SEEK_DOWN) 
  {
    si4703_registers[POWERCFG] &= ~(1<<SEEKUP);//Seek down is the default upon reset
  }
  else 
  {
    si4703_registers[POWERCFG] |= 1<<SEEKUP; //Set the bit to seek up
  }
  si4703_registers[POWERCFG] |= (1<<SEEK); //Start seek
  si4703_updateRegisters(); //Seeking will now start
  //Poll to see if STC is set
  while(1) 
  {
    si4703_readRegisters();
    if((si4703_registers[STATUSRSSI] & (1<<STC)) != 0) break; //Tuning complete!
  }
  si4703_readRegisters();
  int valueSFBL = si4703_registers[STATUSRSSI] & (1<<SFBL); //Store the value of SFBL
  si4703_registers[POWERCFG] &= ~(1<<SEEK); //Clear the seek bit after seek has completed
  si4703_updateRegisters();

  //Wait for the si4703 to clear the STC as well
  while(1) 
  {
    si4703_readRegisters();
    if( (si4703_registers[STATUSRSSI] & (1<<STC)) == 0) break; //Tuning complete!
  }

  if(valueSFBL) 
  { //The bit was set indicating we hit a band limit or failed to find a station
  //Hit limit of band during seek
    return(FAIL);
  }
  //Tuning complete!
  return(SUCCESS);
}
//Write the current 9 control registers (0x02 to 0x07) to the Si4703
//It's a little weird, you don't write an I2C addres
//The Si4703 assumes you are writing to 0x02 first, then increments
byte si4703_updateRegisters(void) 
{
  Wire.beginTransmission(SI4703);
  //A write command automatically begins with register 0x02 so no need to send a write-to address
  //First we send the 0x02 to 0x07 control registers
  //In general, we should not write to registers 0x08 and 0x09
  for(int regSpot = 0x02 ; regSpot < 0x08 ; regSpot++) 
  {
    byte high_byte = si4703_registers[regSpot] >> 8;
    byte low_byte = si4703_registers[regSpot] & 0x00FF;
    Wire.write(high_byte); //Upper 8 bits
    Wire.write(low_byte); //Lower 8 bits
  }
  //End this transmission
  byte ack = Wire.endTransmission();
  si4703_readRegisters();
  return(SUCCESS);
}
//Read the entire register control set from 0x00 to 0x0F
void si4703_readRegisters(void)
{
  //Si4703 begins reading from upper register of 0x0A and reads to 0x0F, then loops to 0x00.
  Wire.requestFrom(SI4703, 32); //We want to read the entire register set from 0x0A to 0x09 = 32 bytes.
  while(Wire.available() < 32) ; //Wait for 16 words/32 bytes to come back from slave I2C device
  //We may want some time-out error here
  //Remember, register 0x0A comes in first so we have to shuffle the array around a bit
  for(int x = 0x0A ; ; x++) 
  { //Read in these 32 bytes
    if(x == 0x10) x = 0; //Loop back to zero
    si4703_registers[x] = Wire.read() << 8;
    si4703_registers[x] |= Wire.read();
    if(x == 0x09) break; //We're done!
  }
}
void lcd_display()
{
  Wire.beginTransmission(lcd_addr);
  lcd.setCursor(col,row);
  lcd.print(txt);
  Wire.endTransmission();
}
void BT_receive()
{
  // data from BT
  txt="";
  while (BT.available()>0)
  {   
    txt+=char(BT.read());
    delay(2);
  } 
}
void BT_send()
{
  if (flg_BT_connected )// just if BT connected the test is used here to avoid using it for each call to BT_send()
  {
    txt="//"+txt;// the "//" is a flag for Android App
    BT.print(txt);
    txt="";   
    temp_control_loop=millis();
    // ***************************control loop to wait ack from Android*******************************
    flg_BT_ack=false;
    while(millis()-temp_control_loop<800 && txt!="ok")
    {
      if (BT.available())
      {
        while (BT.available() )
        {   
          txt+=char(BT.read());
          delay(2);
        }
      }
    } 
    if (txt=="ok")
    {
      flg_BT_ack=true;
      count_BT_error=0;
    }
    else
    {
      count_BT_error+=1;      
    }
    if (count_BT_error==max_BT_attempts)// if "max_BT_attempts" of receiving ack from Android App fail then BT is declared disconnected
    {
      txt="bye";
      decode_txt();
      count_BT_error=0;
    }
    txt="";
  }
}
void set_vol()
{
  txt="v";
  txt+=String(current_vol);
  BT_send(); 
  si4703_registers[SYSCONFIG2] &=0xFFF0; //Clear volume bits
  si4703_registers[SYSCONFIG2] |= current_vol; //Set new volume
  si4703_updateRegisters();
  row=1;
  col=7;
  txt="   ";
  lcd_display();
  txt=String(current_vol);
  lcd_display();
  i=current_vol;
  k=addr_vol;
  put_num(); 
}
void get_num()
{
  // input k as EEPROM address offset, output i 
  i=EEPROM.read(k); 
}
void put_num()
{
  EEPROM.write(addr_vol+k,i);//addr for frequency is mem_addr(addr of volume)+1
}
void radio_status()
{
  //check weither current channel is a preselected channel, returns index or 99 if not
  current_index=99;  
  for (k=first_pref;k<first_pref+pref_index;k++)
  {
    get_num();
    if (i==currentChannel-offset_frequency)
    {
      current_index=k-first_pref;
      break;
    }    
  }
  // check the radio status
  si4703_readRegisters();
  flg_stereo=0;
  if(si4703_registers[STATUSRSSI] & (1<<STEREO))
  { 
    flg_stereo=true;
  }
  rssi = si4703_registers[STATUSRSSI] & 0x00FF; //Mask in RSSI 
  // and display the values and update the android App 
  // display current channel
  if (current_index==99)
  {
    radio_mem="No";
  }
  else
  {
    radio_mem=String(current_index);
  }
  if ((radio_mem!=prev_mem)||flg_force)
  {
    prev_mem=radio_mem;
    row=2;
    col=4;
    txt="   ";
    lcd_display();
    txt=radio_mem;
    lcd_display();   
    txt="m"+radio_mem;
    BT_send();
  }
  // display mode
  if ((flg_stereo!=prev_flg)||flg_force)//just modify display if value has changed
  {
    row=2;
    col=13;
    txt="       ";
    lcd_display();    
    prev_flg=flg_stereo;
    if (flg_stereo)
    {
      txt="stereo";
    }
    else
    {
      txt="Mono";
    }
    lcd_display();
    txt="s"+txt;
    BT_send();
  }
  temp_tune=millis();
  if ((rssi!=prev_rssi)||flg_force)//just modify display if value has changed
  {
    prev_rssi=rssi;
    row=0;
    col=16;
    txt="    ";
    lcd_display();
    txt=String(rssi);
    lcd_display();
    txt="l"+txt;
    BT_send();
  }
  flg_force=false;
}
void get_RDS ()
{
  if(!flg_RDS)
  {
    flg_RDS=true;
  }
}
void get_RT()
{
  si4703_readRegisters();
  flg_RDS=false; // enable GPIO2 interrupt
  gtype=si4703_registers[RDSB]>>11;
  if (gtype==4 || gtype==5)// means text blocks
  {          
    nb_char=(64* (gtype==4)+32*(gtype==5));
    text_pointer=(si4703_registers[RDSB]&0xf)<<2;//4 bytes block index from 0 to 15 => 0 to 64 char
    mess_index=(si4703_registers[RDSB]&0x10)>>4; // set to 0 or 1 when changes => new frame of RDS text
    if (!new_txt)
    {
      //get the 4 char's from RDSC,RDSD
      c=si4703_registers[RDSC]>>8;
      if (c=='\r')
      {
        //means end of the current message
        c=0;
        new_txt=true;
      }    
      text_buffer[text_pointer]=c;
      c=si4703_registers[RDSC]&0xff;
      if (c==0xD)
      {
        c=0;
        new_txt=true;
      }
      text_buffer[text_pointer+1]=c;
      c=si4703_registers[RDSD]>>8;
      if (c=='\r')
      {
        c=0;
        new_txt=true;
      }
      text_buffer[text_pointer+2]=c;
      c=si4703_registers[RDSD]&0xff;
      if (c=='\r')
      {
        c=0;
        new_txt=true;
      }
      text_buffer[text_pointer+3]=c;
      block_count+=1;
      if (block_count==16)
      {
        // all blocks are red  Radio text is available
        new_txt=true;     
      }
    }
    if (mess_index!=prev_index)
    {
      new_txt=true;// in case mess A/B is used by the radio then it means new message
      prev_index=mess_index;
    }    
    if (new_txt)
    {
      new_txt=false;
      block_count=0;// reset blocks count (a full message is 16 blocks of 4 bytes)
      global_RDS="";
      space_count=0;
      for(j=0;j<nb_char;j++)
      {
        global_RDS +=text_buffer[j];
        if (text_buffer[j]=='\0')
        {
          j=nb_char;
        }
        if( int(text_buffer[j])==32)// some radio stations have the starnge habit to fill with spaces a text shorter than 64 char's
        {
          space_count+=1;
          if (space_count==3)
          {
            j=nb_char;                 
          }
        }
        else
        {
          space_count=0;
        }
      }
      memset(text_buffer,0,nb_char);
      if (global_RDS !=prev_RDS)
      {   
        prev_RDS=global_RDS;
        lcd_col=0;
        flg_print_RDS=true;
      }
    }
  }               
} 
void RDS_scroll()// humm!! with LCD display on I2C we can't speak of soft scrolling (next time I'll use an OLED screen
{
  row=3;
  col=0;
  txt=prev_RDS.substring(lcd_col,20+lcd_col);
  lcd_display();
  lcd_col+=1;
  if (lcd_col==prev_RDS.length()-20)
  {
    lcd_col=0;
  }
  scroll_temp=millis();
}
void decode_txt()
{
  if (txt=="Hello")// message from android App when connected
  {
    flg_BT_connected=true;
    digitalWrite(BT_pin,HIGH);
    delay(100);
    txt="v";
    txt+=String(current_vol);
    BT_send();      
    i=readChannel();
    txt="r";
    txt+=global_RDS;
    BT_send();
    flg_force=true;
    radio_status();          
  }
  if(txt == "sav")  
  {
    flg_disp=false;
    i=readChannel()-offset_frequency;
    k=first_pref+pref_index;
    put_num();
    pref_index+=1;
    k=pref_nxtindex;
    i=pref_index;
    put_num();
    flg_disp=true;
    radio_status();
    // send current_index to android app
    current_index=pref_index-1;
  }
  if(txt == "prefd")  
  {
    if (current_index!=99)
    {
      if (current_index>0)
      {
        current_index-=1;
      }
    }
    else
    {
      current_index=0;
    }
    k=first_pref+current_index;   
    get_num();
    currentChannel=i+offset_frequency;
    gotoChannel(currentChannel);
    i = readChannel();
    // send current_index to android app
    radio_status();
    raz_RDS();
    prev_index=3;
  }  
  if(txt == "prefu")  
  {
    if (current_index!=99)
    {
      if (current_index<pref_index-1)
      {
        current_index+=1;
      }
    }
    else
    {
      current_index=0;
    }
    k=first_pref+current_index;
    get_num();
    currentChannel=i+offset_frequency;
    gotoChannel(currentChannel);
    i = readChannel();// to display, send to android app and save the current channel in eeprom
    radio_status();
    raz_RDS();
    prev_index=3;
  }               
  if(txt == "reset") // reset off preselected radio's
  {
    pref_index=0;
    current_index=0;
    k=pref_nxtindex;
    i=pref_index;
    put_num();
  }
  if(txt == "su")
  { 
     seek(SEEK_UP);
     currentChannel=readChannel();
     i=currentChannel-offset_frequency;
     k=1;
     put_num();
     radio_status();
     raz_RDS();         
  }
  if(txt == "sd") 
  {
     seek(SEEK_DOWN);
     currentChannel=readChannel();
     i=currentChannel-offset_frequency;
     k=1;
     put_num();
     radio_status();
     raz_RDS();                
  }
  if(txt == "v+" )
  {
    si4703_readRegisters(); //Read the current register set
    current_vol = si4703_registers[SYSCONFIG2] & 0x000F; //Read the current volume level
    if(current_vol < 15)
    {
      current_vol+=1; //Limit max volume to 0x000F
    }
    set_vol();
  }
  if(txt == "v-") 
  {
    si4703_readRegisters(); //Read the current register set
    current_vol = si4703_registers[SYSCONFIG2] & 0x000F; //Read the current volume level
    if(current_vol > 0) 
    {
      current_vol-=1; 
    }
    set_vol();     
  }
  if(txt == "f+") 
  {
    flg_disp=false;      
    currentChannel = readChannel();
    currentChannel += 1; //Increase channel by 100kHz
    gotoChannel(currentChannel);
    flg_disp=true;
    i=readChannel();
    i-=offset_frequency;// one byte is saved only
    k=addr_freq;
    put_num();
    radio_status();
    raz_RDS();
  }
  if(txt == "f-") 
  {
    flg_disp=false;
    currentChannel = readChannel();
    currentChannel -= 1; //Decreage channel by 100kHz
    gotoChannel(currentChannel);
    flg_disp=true;
    i=readChannel();
    i-=offset_frequency;// one byte is saved only
    k=addr_freq;
    put_num();
    radio_status();
    raz_RDS();
  } 
  if(txt == "bye") 
  {
    flg_BT_connected=false;
    digitalWrite(BT_pin,LOW);      
  }
  txt="";
}
void mes_power()
{
  // test power voltage
  temp_tasks=millis();
  val=analogRead(power_mes);
  if (val>750)
  {
    digitalWrite(power_pin,HIGH);
    digitalWrite(battery_pin,LOW);
    txt="pow";
    bat_low=false;
    BT_send();
  }
  else
  {
    if (val>550)// around 8 volts
    {
      digitalWrite(power_pin,LOW);
      digitalWrite(battery_pin,HIGH);
      txt="bat";
      bat_low=false;
      BT_send(); 
    }
    else
    {
      txt="bl" ;
      bat_low=true;
      BT_send();          
    }
  }
}
void raz_RDS()
{
  col=0;
  row=3;
  txt="                    ";
  prev_RDS=txt;
  lcd_display(); 
}

