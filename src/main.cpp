#include <Arduino.h>

/*Pins*/
//Sensors
byte S_ZERO = 34;
byte S_30 = 35;
byte S_100 = 32;
byte S_C1 = 33;
byte S_C2 = 25;
//Debug Pins
byte LED = 2;

/*Data Structures*/
typedef struct
{
    volatile unsigned long int time_in_30;
    volatile unsigned long int time_c_start;
    volatile unsigned long int time_c_end; 
    volatile unsigned long int time_in_100;
    
} packet_ble;

/*Variables*/
typedef enum {BLUETOOTH, RUN, SAVE}  state;
typedef enum {START_, WAIT_30, WAIT_C1, WAIT_C2, WAIT_100, END_RUN} run_state;

state ss = BLUETOOTH;
run_state ss_r = START_;

packet_ble data;
bool interrupt = false;
unsigned long int t_curr = 0;
volatile unsigned long int t_30 = 0, t_c1 = 0, t_c2 = 0, t_100 = 0;
       
/*Functions*/
void isr_30m();
void isr_c1();
void isr_c2();
void isr_100m();

void setup()
{
    Serial.begin(9600);
    pinMode(LED,OUTPUT);
    pinMode(S_ZERO, INPUT_PULLUP);
    pinMode(S_30, INPUT_PULLUP);
    pinMode(S_100, INPUT_PULLUP);
    pinMode(S_C1, INPUT_PULLUP);
    pinMode(S_C2, INPUT_PULLUP);

    digitalWrite(LED, HIGH);
}

void loop()
{   
    switch (ss)
    {
        case BLUETOOTH:  
            ss = RUN;
            ss_r = START_;
            break;
        case RUN:
            switch (ss_r)
            {               
                case START_:
                    t_30 = 0;
                    t_c1 = 0;
                    t_c2 = 0;
                    t_100 = 0;
                    if(digitalRead(S_ZERO))
                    {
                        ss_r = WAIT_30;
                        t_curr = millis();
                    }
                    break;
                case WAIT_30:
                    if (!interrupt && (ss_r == WAIT_30))
                    {
                        interrupt = true;
                        attachInterrupt(digitalPinToInterrupt(S_30), isr_30m, RISING);
                    }
                    else
                    {
                        t_30 = millis() - t_curr;
                        t_c1 = millis() - t_curr;
                        t_c2 = millis() - t_curr;
                        t_100 = millis() - t_curr;
                    }
                    break;
                case WAIT_C1:
                    if (((millis()- t_curr) - t_30 >  1000) && !interrupt && (ss_r == WAIT_C1))
                    {
                        interrupt = true;
                        attachInterrupt(digitalPinToInterrupt(S_C1), isr_c1, RISING);
                    }
                    else
                    {
                        t_c1 = millis() - t_curr;
                        t_c2 = millis() - t_curr;
                        t_100 = millis() - t_curr;
                    }
                    break;
                case WAIT_C2:
                    if (((millis()- t_curr) - t_c1 >  1000) && !interrupt && (ss_r == WAIT_C2))
                    {
                        interrupt = true;
                        attachInterrupt(digitalPinToInterrupt(S_C2), isr_c2, RISING);
                    }
                    else
                    {
                        t_c2 = millis() - t_curr;
                        t_100 = millis() - t_curr;
                    }
                    break;
                case WAIT_100:
                    if (((millis()- t_curr) - t_c2 >  1000) && !interrupt && (ss_r == WAIT_100))
                    {
                        interrupt = true;
                        attachInterrupt(digitalPinToInterrupt(S_100), isr_100m, RISING);
                    }
                    else
                    {
                        t_100 = millis() - t_curr;
                    }
                    break;
                case END_RUN:
                    data.time_in_30 = t_30;
                    data.time_c_start = t_c1;
                    data.time_c_end = t_c2;
                    data.time_in_100 = t_100;
                    ss = BLUETOOTH;
                    break;
            }
            break;
        case SAVE:
            ss = BLUETOOTH;
            break;
    }
}

void isr_30m()
{
    ss_r = WAIT_C1;

    interrupt = false;
    detachInterrupt(digitalPinToInterrupt(S_30));
}

void isr_c1()
{
    ss_r = WAIT_C2;

    interrupt = false;
    detachInterrupt(digitalPinToInterrupt(S_C1));
}

void isr_c2()
{
    ss_r = WAIT_100;

    interrupt = false;
    detachInterrupt(digitalPinToInterrupt(S_C2));
}

void isr_100m()
{
    ss_r = END_RUN;

    interrupt = false;
    detachInterrupt(digitalPinToInterrupt(S_100));
}
