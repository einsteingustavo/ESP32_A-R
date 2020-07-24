#include <Arduino.h>

/*Pins*/
//Sensors
byte S_ZERO = 34;   //Initial sensor
byte S_30 = 35;     //30m sensor
byte S_100 = 25;    //100m sensor
byte S_C1 = 32;     //1st sensor on the corner
byte S_C2 = 33;     //2nd sensor on the corner
//Debug Pins
byte LED = 2;       //LED for debugging

/*Data Structures*/
typedef struct
{
    volatile unsigned long time_in_30;
    volatile unsigned long time_c_start;
    volatile unsigned long time_c_end; 
    volatile unsigned long time_in_100;
    
} packet_ble;       //Package type to send by bluetooth

/*Variables*/
typedef enum {BLUETOOTH, RUN, SAVE}  state;     //Main states
typedef enum {START_, WAIT_30, WAIT_C1, WAIT_C2, WAIT_100, END_RUN} run_state;  //Secondary/run states

state ss = BLUETOOTH;       //Initial main state setting
run_state ss_r = START_;    //Initial secondary/run state setting

packet_ble data;            //Package to send by bluetooth
bool interrupt = false;     //Autoral support boolean to interrupt functions
unsigned long t_curr = 0;   //Current initial time, updated every run
volatile unsigned long t_30 = 0, t_c1 = 0, t_c2 = 0, t_100 = 0;     //Time variables
       
/*Function Prototypes*/
void isr_30m();     //30m interrupt
void isr_c1();      //Start of curve interrupt
void isr_c2();      //End of curve interrupt
void isr_100m();    //100m interrupt

void setup()
{
    Serial.begin(9600);     //Serial with 9600 baud rate
    //Pin setups
    pinMode(LED,OUTPUT);
    pinMode(S_ZERO, INPUT_PULLUP);
    pinMode(S_30, INPUT_PULLUP);
    pinMode(S_100, INPUT_PULLUP);
    pinMode(S_C1, INPUT_PULLUP);
    pinMode(S_C2, INPUT_PULLUP);

    digitalWrite(LED, HIGH);    //LED is at high state
}

void loop()     //Main loop
{   
    switch (ss)     //Switch for main states
    {
        case BLUETOOTH:  
            ss = RUN;
            ss_r = START_;
            break;
        case RUN:
            switch (ss_r)   //Switch for secondary/run states
            {               
                case START_:    //Start of RUN
                    //Reset all variables
                    t_30 = 0;
                    t_c1 = 0;
                    t_c2 = 0;
                    t_100 = 0;
                    if(digitalRead(S_ZERO))     //If the car isn't in front of sensor anymore
                    {
                        ss_r = WAIT_30;         //Trigger the secondary state for waiting 30m sensor   
                        t_curr = millis();      //Initial time
                    }
                    break;
                case WAIT_30:   //Wainting for the car to get trough 30m sensor
                    if (!interrupt && (ss_r == WAIT_30))    //If interrupt isn't active and this is the actual state
                    {
                        interrupt = true;   //Set the support boolean
                        attachInterrupt(digitalPinToInterrupt(S_30), isr_30m, RISING); //Attach interrupt
                    }
                    else
                    {
                        //While in this state these variables will be updated
                        t_30 = millis() - t_curr;
                        t_c1 = millis() - t_curr;
                        t_c2 = millis() - t_curr;
                        t_100 = millis() - t_curr;
                    }
                    break;
                case WAIT_C1:   //Wainting for the car to get trough 1st sensor of the curve
                    if (((millis()- t_curr) - t_30 >  1000) && !interrupt && (ss_r == WAIT_C1)) //If at least one secont has passed and interrupt isn't active and this is the actual state
                    {
                        interrupt = true;   //Set the support boolean
                        attachInterrupt(digitalPinToInterrupt(S_C1), isr_c1, RISING);   //Attach interrupt
                    }
                    else
                    {
                        //While in this state these variables will be updated
                        t_c1 = millis() - t_curr;
                        t_c2 = millis() - t_curr;
                        t_100 = millis() - t_curr;
                    }
                    break;
                case WAIT_C2:   //Wainting for the car to get trough 2nd sensor of the curve
                    if (((millis()- t_curr) - t_c1 >  1000) && !interrupt && (ss_r == WAIT_C2)) //If at least one secont has passed and interrupt isn't active and this is the actual state
                    {
                        interrupt = true;   //Set the support boolean
                        attachInterrupt(digitalPinToInterrupt(S_C2), isr_c2, RISING);   //Attach interrupt
                    }
                    else
                    {   
                        //While in this state these variables will be updated
                        t_c2 = millis() - t_curr;
                        t_100 = millis() - t_curr;
                    }
                    break;
                case WAIT_100:  //Wainting for the car to get trough 100m sensor
                    if (((millis()- t_curr) - t_c2 >  1000) && !interrupt && (ss_r == WAIT_100))    //If at least one secont has passed and interrupt isn't active and this is the actual state
                    {
                        interrupt = true;   //Set the support boolean
                        attachInterrupt(digitalPinToInterrupt(S_100), isr_100m, RISING);    //Attach interrupt
                    }
                    else
                    {
                        //While in this state this variables will be updated
                        t_100 = millis() - t_curr;
                    }
                    break;
                case END_RUN:   //The car has passed by 100m sensor, run is ended
                    //Update all variables in the data package
                    data.time_in_30 = t_30;
                    data.time_c_start = t_c1;
                    data.time_c_end = t_c2;
                    data.time_in_100 = t_100;
                    ss = BLUETOOTH;     //Triggger the Bluetooth main state
                    break;
            }
            break;
        case SAVE:      //This main state is to save a file in SD that contains some data packages
            ss = BLUETOOTH;     //Triggger the Bluetooth main state
            break;
    }
}

void isr_30m()  //30m interrupt function
{
    ss_r = WAIT_C1;     //Trigger the secondary state for waiting 1st sensor of the curve

    interrupt = false;  //Reset support boolean
    detachInterrupt(digitalPinToInterrupt(S_30));   //Detach Interrupt
}

void isr_c1()   //Start of curve interrupt function
{
    ss_r = WAIT_C2;     //Trigger the secondary state for waiting 2nd sensor of the curve

    interrupt = false;  //Reset support boolean
    detachInterrupt(digitalPinToInterrupt(S_C1));   //Detach Interrupt
}

void isr_c2()   //End of curve interrupt function
{
    ss_r = WAIT_100;    //Trigger the secondary state for waiting 100m sensor

    interrupt = false;  //Reset support boolean
    detachInterrupt(digitalPinToInterrupt(S_C2));   //Detach Interrupt
}

void isr_100m() //100m interrupt function
{
    ss_r = END_RUN;     //Trigger the secondary state for end the RUN

    interrupt = false;  //Reset support boolean
    detachInterrupt(digitalPinToInterrupt(S_100));  //Detach Interrupt
}
