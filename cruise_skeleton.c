/* Cruise control skeleton for the IL 2206 embedded lab
 *
 * Maintainers:  Rodolfo Jordao (jordao@kth.se), George Ungereanu (ugeorge@kth.se)
 *
 * Description
 *
 *   In this file you will find the "model" for the vehicle that is being simulated on top
 *   of the RTOS and also the stub for the control task that should ideally control its
 *   velocity whenever a cruise mode is activated.
 *
 *   The missing functions and implementations in this file are left as such for
 *   the students of the IL2206 course. The goal is that they get familiriazed with
 *   the real time concepts necessary for all implemented herein and also with Sw/Hw
 *   interactions that includes HAL calls and IO interactions.
 *
 *   If the prints prove themselves too heavy for the final code, they can
 *   be exchanged for alt_printf where hexadecimals are supported and also
 *   quite readable. This modification is easily motivated and accepted by the course
 *   staff.
 */
#include <stdio.h>
#include "system.h"
#include "includes.h"
#include "altera_avalon_pio_regs.h"
#include "sys/alt_irq.h"
#include "sys/alt_alarm.h"
#include "altera_avalon_performance_counter.h"


#define DEBUG 1

#define HW_TIMER_PERIOD 1 /* 1ms */

/* Button Patterns */

#define GAS_PEDAL_FLAG      0x08
#define BRAKE_PEDAL_FLAG    0x04
#define CRUISE_CONTROL_FLAG 0x01 //change to this button because KEY 3 doesn't work.
/* Switch Patterns */

#define TOP_GEAR_FLAG       0x00000002
#define ENGINE_FLAG         0x00000001
//TASK 4.5 extra task definition
#define EXTRA_TASK_FLAG     (0x3F<<4) // SW4 to SW9 buttons

/* LED Patterns */

#define LED_RED_0 0x00000001 // Engine
#define LED_RED_1 0x00000002 // Top Gear

#define LED_GREEN_0 0x0001 // Cruise Control activated
#define LED_GREEN_2 0x0002 // Cruise Control Button
#define LED_GREEN_4 0x0010 // Brake Pedal
#define LED_GREEN_6 0x0040 // Gas Pedal
#define POSITION_CLEAR_FLAG (0x3f << 12)

/*
 * Definition of Tasks
 */

#define TASK_STACKSIZE 2048

OS_STK StartTask_Stack[TASK_STACKSIZE]; 
OS_STK ControlTask_Stack[TASK_STACKSIZE]; 
OS_STK VehicleTask_Stack[TASK_STACKSIZE];
OS_STK ButtonIO_Stack[TASK_STACKSIZE];  //4.3
OS_STK SwitchIO_Stack[TASK_STACKSIZE];  //4.3

// 4.5
OS_STK WatchdogTask_Stack[TASK_STACKSIZE];
OS_STK OverloadTask_Stack[TASK_STACKSIZE];
OS_STK ExtraTask_Stack[TASK_STACKSIZE];

// Task Priorities
#define STARTTASK_PRIO    5 //high priority
#define WATCHDOGTASK_PRIO   6  // should have highest(except for start) in order to signal at end of hyperperiod
#define EXTRATASK_PRIO      9 // higher prio than other tasks except for start and wtachdog; to stall properly
#define VEHICLETASK_PRIO  10
#define CONTROLTASK_PRIO  12
#define BUTTONIO_PRIO   14  //4.3
#define SWITCHIO_PRIO   15 //4.3
#define OVERLOADTASK_PRIO   17 // overload low priortity task that runs in the background

// Task Periods
#define CONTROL_PERIOD  300
#define VEHICLE_PERIOD  300
#define BUTTON_PERIOD  100
#define SWITCH_PERIOD  100

// 4.5
#define EXTRA_PERIOD  300
#define HYPERPERIOD   300 // lcm of all tasks except watch-d and overload
#define OVERLOAD_PERIOD 300 
#define WATCHDOGPERIOD_PERIOD  HYPERPERIOD // lcm 
//MACRO to convert ticks to MS; min val 50ms
#define MS_IN_TICKS(ms) ((ms)/HW_TIMER_PERIOD) // each tick 50 ms, one period 2 ticks or smth

/*
 * Definition of Kernel Objects 
 */

// Mailboxes
OS_EVENT *Mbox_Throttle;
OS_EVENT *Mbox_Velocity;
OS_EVENT *Mbox_Brake;
OS_EVENT *Mbox_BrakeControl;
OS_EVENT *Mbox_Engine; 

OS_EVENT *Mbox_EngineControl;//used in Switch/control Task to read from switch and send to engine
OS_EVENT *Mbox_Gear; // used for input Gear (Low/High)
OS_EVENT *Mbox_Cruise_Control; //save target velocity while cruise control is ON
OS_EVENT *Mbox_Gas_Pedal; //used for gas pedal value

//task 4.5
OS_EVENT *Mbox_Watchdog_Signal; //mailbox for watchdog ok signal; 1-signal received
OS_EVENT *Mbox_ExtraTask_Utilization; //mailbox for extra task to determine utilization -remember 6 bit value

// Semaphores
OS_EVENT *Vehicle_Sem;    //4.2
OS_EVENT *Control_Sem;    //4.2
OS_EVENT *ButtonIO_Sem;   //4.2
OS_EVENT *SwitchIO_Sem;   //4.2
//TASK 4.5
OS_EVENT *Watchdog_Sem;
OS_EVENT *Overload_Sem;
OS_EVENT *Extra_Task_Sem;

// SW-Timer
OS_TMR *Vehicle_Timer;    //4.2
OS_TMR *Control_Timer;    //4.2
OS_TMR *ButtonIO_Timer;   //4.2
OS_TMR *SwitchIO_Timer;   //4.2
//TASK 4.5
OS_TMR *Watchdog_Timer;
OS_TMR *Overload_Timer;
OS_TMR *Extra_Task_Timer;

/*
Timer callback functions for all Tasks. it is used to
//signal a task that the timer expired, or perform other function.
*/

OS_TMR_CALLBACK Vehicle_Timer_Callback(void *ptmr, void *parg) 
{
  INT8U err; 
   //printf("Vechicle task callback\n");
  err = OSSemPost(Vehicle_Sem); 
} 

OS_TMR_CALLBACK Control_Timer_Callback(void *ptmr, void *parg)
{
  INT8U err; 
   //printf("Control task callback\n");
  err = OSSemPost(Control_Sem); 
} 

OS_TMR_CALLBACK ButtonIO_Timer_Callback(void *ptmr, void *parg)
{
  INT8U err; 
   //printf("Button task callback\n");
  err = OSSemPost(ButtonIO_Sem); 
} 

OS_TMR_CALLBACK SwitchIO_Timer_Callback(void *ptmr, void *parg)
{
  INT8U err; 
   //printf("Switch task callback\n");
  err = OSSemPost(SwitchIO_Sem); 
} 

//TASK 4.5
OS_TMR_CALLBACK Watchdog_Timer_Callback(void *ptmr, void *parg) 
{
  INT8U err; 
   //printf("watchdog task callback\n");
  err = OSSemPost(Watchdog_Sem); 
} 

OS_TMR_CALLBACK Overload_Timer_Callback(void *ptmr, void *parg)
{
  INT8U err; 
   printf("Overload task callback\n");
  err = OSSemPost(Overload_Sem); 
  printf("err: %d\n", err );
} 

OS_TMR_CALLBACK Extra_Task_Timer_Callback (void *ptmr, void *parg)
{
  INT8U err;
    printf ("Extra task callback\n");
    err = OSSemPost(Extra_Task_Sem);
}

/*
 * Types
 */
enum active {on = 2, off = 1};

/*
 * Global variables
 */
int delay; // Delay of HW-timer 
INT16U led_green = 0; // Green LEDs
INT32U led_red = 0;   // Red LEDs
int clicks = 0;
/*
 * Helper functions
 buttons_pressed & switches_pressed were in Original File.
 */
int buttons_pressed(void)
{
  return ~IORD_ALTERA_AVALON_PIO_DATA(D2_PIO_KEYS4_BASE);  //DE2 OR D2????
}

int switches_pressed(void)
{
  return IORD_ALTERA_AVALON_PIO_DATA(DE2_PIO_TOGGLES18_BASE);    
}

/*
 * ISR for HW Timer (ORIGINAL FILE)
 */
alt_u32 alarm_handler(void* context)
{
  OSTmrSignal(); /* Signals a 'tick' to the SW timers */

  return delay;
}


static int b2sLUT[] = {
  0x40, //0
  0x79, //1
  0x24, //2
  0x30, //3
  0x19, //4
  0x12, //5
  0x02, //6
  0x78, //7
  0x00, //8
  0x18, //9
  0x3F, //-
};

/*
 * convert int to seven segment display format
 */
int int2seven(int inval){
  return b2sLUT[inval];
}

/*
 * output current velocity on the seven segement display
 ORIGINAL FILE
 */
void show_velocity_on_sevenseg(INT8S velocity)
{
  int tmp = velocity;
  int out;
  INT8U out_high = 0;
  INT8U out_low = 0;
  INT8U out_sign = 0;

  if(velocity < 0)
  {
    out_sign = int2seven(10);
    tmp *= -1;
  }
  else
  {
    out_sign = int2seven(0);
  }

  out_high = int2seven(tmp / 10);
  out_low = int2seven(tmp - (tmp/10) * 10);

  out = int2seven(0) << 21 |
    out_sign << 14 |
    out_high << 7  |
    out_low;
  IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_HEX_LOW28_BASE,out);
}

/*
 * shows the target velocity on the seven segment display (HEX5, HEX4)
 * when the cruise control is activated (0 otherwise)
 FUNCTION IMPLEMENTED BY US. BASED ON THE STRUCTURE
 OF SHOW_VELOCITY_ON_SEVENSEG
 */

void show_target_velocity(INT8U target_vel) //needs to be implemented further...
{
    int tmp = target_vel;
    int out;
    INT8U out_high = 0;
    INT8U out_low = 0;
    
    out_high = int2seven(tmp / 10);
    out_low = int2seven(tmp - (tmp/10) * 10);
    
    out = int2seven(0) << 21 | int2seven(0) << 14 | out_high << 7  | out_low;
    
    IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_HEX_HIGH28_BASE, out);
}

/*
 * indicates the position of the vehicle on the track with the four leftmost red LEDs
 * LEDR17: [0m, 400m)
 * LEDR16: [400m, 800m)
 * LEDR15: [800m, 1200m)
 * LEDR14: [1200m, 1600m)
 * LEDR13: [1600m, 2000m)
 * LEDR12: [2000m, 2400m]
 */

/*
show_position function implemented by us.
Using switch-case structure to identify position of 
the car on the track.
*/
void show_position(INT16U position)
{
  setLed(DE2_PIO_REDLED18_BASE,POSITION_CLEAR_FLAG,0); 
  switch(position)
  {
    case 0  ... 399:    
      setLed(DE2_PIO_REDLED18_BASE,(POSITION_CLEAR_FLAG & ~(0x1F<<12)),1); 
    break; 

    case 400 ... 799:  
      setLed(DE2_PIO_REDLED18_BASE,(POSITION_CLEAR_FLAG & ~(0x0F<<12)),1); 
    break; 

    case 800 ... 1199: 
      setLed(DE2_PIO_REDLED18_BASE,(POSITION_CLEAR_FLAG & ~(0x07<<12)),1); 
     break;
    
    case 1200 ... 1599: 
      setLed(DE2_PIO_REDLED18_BASE,(POSITION_CLEAR_FLAG & ~(0x03<<12)),1); 
    break; 
    
    case 1600 ... 1999: 
      setLed(DE2_PIO_REDLED18_BASE,(POSITION_CLEAR_FLAG & ~(0x01<<12)),1); 
    break;

    case 2000 ... 2400:
     setLed(DE2_PIO_REDLED18_BASE,POSITION_CLEAR_FLAG,1); 
    break;    
  }
}

//creating the below function to set the status of LED
void setLed(int base_addr,int mask, int val)
{
  int read_led_data = IORD_ALTERA_AVALON_PIO_DATA(base_addr);
  if(val)
    read_led_data |= mask; //|=Bitwise inclusive OR and assignment operator.
  else
    read_led_data &= ~mask;
  IOWR_ALTERA_AVALON_PIO_DATA(base_addr,read_led_data);
}


/*
 * The task 'VehicleTask' is the model of the vehicle being simulated. It updates variables like
 * acceleration and velocity based on the input given to the model.
 * 
 * The car model is equivalent to moving mass with linear resistances acting upon it.
 * Therefore, if left one, it will stably stop as the velocity converges to zero on a flat surface.
 * You can prove that easily via basic LTI systems methods.
 
 NO BIG CHANGES IN THIS TASK FROM US, APART FROM SOFT TIMER
 FOR TASK 4.2 AND SOME SYNTAX ERRORS.
 */

void VehicleTask(void* pdata)
{ 
  // constants that should not be modified
  const unsigned int wind_factor = 1;
  const unsigned int brake_factor = 4;
  const unsigned int gravity_factor = 2;
  // variables relevant to the model and its simulation on top of the RTOS
  INT8U err;  
  void* msg;
  INT8U* throttle; 
  INT16S acceleration;  
  INT16U position = 0; 
  INT16S velocity = 0; 
  enum active brake_pedal = off;
  enum active engine = on;

  printf("Vehicle task created!\n");

  while(1)
  {    
    err = OSMboxPost(Mbox_Velocity, (void *) &velocity); //task sends message to another task

    //OSTimeDlyHMSM(0,0,0,VEHICLE_PERIOD); //REMOVE FOR TASK 4.2
    
    //use soft timer to implement periodic task 4.2
    OSSemPend(Vehicle_Sem,0,&err);

    /* Non-blocking read of mailbox: 
       - message in mailbox: update throttle
       - no message:         use old throttle
       */
      msg = OSMboxPend(Mbox_Throttle, 1, &err); 
      if (err == OS_NO_ERR) 
        *throttle = *((INT8U*) msg);
        /* Same for the brake signal that bypass the control law */
      msg = OSMboxPend(Mbox_Brake, 1, &err); 
      if (err == OS_NO_ERR) 
        brake_pedal = *((enum active*) msg);
        /* Same for the engine signal that bypass the control law */
      msg = OSMboxPend(Mbox_Engine, 1, &err); 
      if (err == OS_NO_ERR) 
        engine = *((enum active*) msg);

      // vehichle cannot effort more than 80 units of throttle
      if (*throttle > 80) *throttle = 80;
      // brakes + wind
      if (brake_pedal == off)
      {
      // wind resistance
      acceleration = - wind_factor*velocity;
      // actuate with engines
      if (engine == on)
        acceleration += (*throttle);

      // gravity effects
      if (400 <= position && position < 800)
        acceleration -= gravity_factor; // traveling uphill
      else if (800 <= position && position < 1200)
        acceleration -= 2*gravity_factor; // traveling steep uphill
      else if (1600 <= position && position < 2000)
        acceleration += 2*gravity_factor; //traveling downhill
      else if (2000 <= position)
        acceleration += gravity_factor; // traveling steep downhill
    }
    // if the engine and the brakes are activated at the same time,
    // we assume that the brake dynamics dominates, so both cases fall
    // here.
    else 
      acceleration = - brake_factor*velocity;

    printf("\nPosition: %d m\n", position);
    printf("Velocity: %d m/s\n", velocity);
    printf("Accell: %d m/s2\n", acceleration);
    printf("Throttle: %d V\n", *throttle);
    

    position = position + velocity * VEHICLE_PERIOD / 1000;
    velocity = velocity  + acceleration * VEHICLE_PERIOD / 1000.0;
    

    // reset the position to the beginning of the track
    if(position > 2400)
      position = 0;
      show_velocity_on_sevenseg((INT8S) velocity);
  }
}

/*
CREATION AND IMPLEMENTATION OF BUTTONIO FUNCTIONS
*/
void ButtonIO(void*pdata)
{
  INT8U err;
  void* msg;
  
  int button_selection=0; //user's input for keys (on/off)

  enum active gas_pedal = off;
  enum active top_gear = off;
  enum active brake_pedal = off;
  enum active cruise_control = off;
  enum active prev_cruise_control = off;

  printf ("ButtonIO task created!\n");

  while (1)
  {
    OSSemPend(ButtonIO_Sem, 0, &err); //use soft timer for periodic task
    button_selection=buttons_pressed();

    // reading the input of the user and checking if it matches the corresponding flag
    //GAS KEY 3 - LED 6
    if (button_selection & GAS_PEDAL_FLAG)
      {
        gas_pedal = on;
        setLed(DE2_PIO_GREENLED9_BASE,LED_GREEN_6,1); //switch ON GREEN LED
      }
    else
      {
        gas_pedal = off;
        setLed(DE2_PIO_GREENLED9_BASE,LED_GREEN_6,0); //switch OFF GREEN LED
      }
    //BRAKE KEY 2 - LED 4
    if (button_selection & BRAKE_PEDAL_FLAG)
      {
        brake_pedal = on;
        setLed(DE2_PIO_GREENLED9_BASE,LED_GREEN_4,1); //switch ON GREEN LED
      }
    else
      {
        brake_pedal = off;
        setLed(DE2_PIO_GREENLED9_BASE,LED_GREEN_4,0); //switch OFF GREEN LED
      }
    //CRUISE CONTROL KEY 3 - LED 2
    if (button_selection & CRUISE_CONTROL_FLAG)
      {
        cruise_control = on;
        setLed(DE2_PIO_GREENLED9_BASE,LED_GREEN_2,1); //switch ON GREEN LED
      }
    else
      {
        cruise_control = off;
        setLed(DE2_PIO_GREENLED9_BASE,LED_GREEN_2,0); //switch OFF GREEN LED
      }
    // send messages to mailboxes
    if(cruise_control == on && prev_cruise_control == off)
      {
        err = OSMboxPost(Mbox_Cruise_Control, (void *)&cruise_control);
        printf("TRYING TO TOGGLE CRUISE CONTROL\n");
      }
    prev_cruise_control = cruise_control;
    err = OSMboxPost(Mbox_Gas_Pedal, (void *)&gas_pedal);
    err = OSMboxPost(Mbox_Brake, (void *)&brake_pedal);
    err = OSMboxPost(Mbox_BrakeControl, (void *)&brake_pedal);
    //printf("Buttons %d\n",buttons_pressed());
  }
}

/*
CREATION AND IMPLEMENTATION OF SWITCH FUNCTION
*/

void SwitchIO(void* pdata)
{
  INT8U err;
  void* msg;
  enum active engine = off;
  enum active top_gear = off;
  int leds=0;
  int utilisation = 0; //USED FOR TASK 4.5

  printf("Switch Task Created!\n");
  while(1)
  {
    // Use soft timer to implement periodic task
    OSSemPend(SwitchIO_Sem, 0, &err);  
    // Read switches
    int switch_values = switches_pressed();

    // Check if engine is active
    if ((switch_values & ENGINE_FLAG))
    {
      engine = on;
      setLed(DE2_PIO_REDLED18_BASE,LED_RED_0,1);
    }
    else
    {
      engine = off;
      setLed(DE2_PIO_REDLED18_BASE,LED_RED_0,0);
    }
 
    // Check if gear is high
    if (switch_values & TOP_GEAR_FLAG)
    {
      top_gear = on;
          setLed(DE2_PIO_REDLED18_BASE,LED_RED_1,1);
    }
    else
    { 
      top_gear = off;
          setLed(DE2_PIO_REDLED18_BASE,LED_RED_1,0);

    }
    // for 4.5
    
    if (switch_values & EXTRA_TASK_FLAG)
      utilisation = (switch_values & EXTRA_TASK_FLAG) >> 4; // shift back to get value 
    else
      utilisation = 0;
    // send messages to mailboxes
    err = OSMboxPost(Mbox_EngineControl, (void *)&engine);
    err = OSMboxPost(Mbox_Gear, (void *)&top_gear);
    // TASK 4.5
    err = OSMboxPost(Mbox_ExtraTask_Utilization, (void *)&utilisation);
  }  
}


/*
 * The task 'ControlTask' is the main task of the application. It reacts
 * on sensors and generates responses.
 */

//////////////////////////////////////////////////////////////////////////////////
void ControlTask(void* pdata)
{
  INT8U err;
  INT8U throttle = 0; /* Value between 0 and 80, which is interpreted as between 0.0V and 8.0V */
  void* msg;
  INT16S current_velocity;

 
  INT16S target_velocity; //extra
  INT16U position = 0;    //extra

  enum active gas_pedal = off;
  enum active brake_pedal = off;
  enum active top_gear = off;
  enum active cruise_control = off; 

  printf("Control Task created!\n");

  while(1)
  {

    OSSemPend(Control_Sem, 0 ,&err);    
    // Here you can use whatever technique or algorithm that you prefer to control
    // the velocity via the throttle. There are no right and wrong answer to this controller, so
    // be free to use anything that is able to maintain the cruise working properly. You are also
    // allowed to store more than one sample of the velocity. For instance, you could define
    //
    // INT16S previous_vel;
    // INT16S pre_previous_vel;
    // ...
    //
    // If your control algorithm/technique needs them in order to function. 
    
    //Read gas pedal value
    msg = OSMboxPend (Mbox_Gas_Pedal, 0, &err);
    gas_pedal = *((enum active*)msg);

    //Read brake value
    msg = OSMboxPend(Mbox_BrakeControl, 0, &err);
    brake_pedal = *((enum active*)msg);

    //Read gear value
    msg = OSMboxPend(Mbox_Gear, 0, &err);
    top_gear = *((enum active*)msg);

    //Read current velocity
    msg = OSMboxPend(Mbox_Velocity, 0, &err);  //was already in the original code
    current_velocity = *((INT16S*)msg);

  //at this point we need to read from switch and send the input data to the engine
    msg = OSMboxPend(Mbox_EngineControl, 1, &err);
    if (err == OS_NO_ERR)
    {
      if (current_velocity == 0)
        {
          OSMboxPost(Mbox_Engine,msg);
            if (*((enum active*)msg) == on)
              setLed(DE2_PIO_REDLED18_BASE,LED_RED_0,1);
              else if(*((enum active*)msg) == off)
              setLed(DE2_PIO_REDLED18_BASE,LED_RED_0,0);
        }
      }
    // save velocity for cruise control to use it

    msg = OSMboxPend(Mbox_Cruise_Control, 1, &err);
    if (err==OS_NO_ERR)
      {
        if (cruise_control==on)
          {
            cruise_control = off;
            target_velocity = 0;
            setLed(DE2_PIO_GREENLED9_BASE, LED_GREEN_2, 0);
          }
          // TURN ON LED FOR CRUISE CONTROL
          else if (current_velocity>=20 && top_gear==on)
          {
            target_velocity = current_velocity;
            cruise_control = on;
            setLed(DE2_PIO_GREENLED9_BASE, LED_GREEN_2, 1);
          }
          clicks = 0;
      }    
    //set maximum value for gas at 80
    if(gas_pedal == on) 
      throttle = 80;
    else if (cruise_control == off)
      throttle = 0;

    if(cruise_control == on)
    {
      throttle = target_velocity;
    }
    //deactivate cruise control if we change to LOW gear
    //OR if we hit the break!
    if (top_gear == off || brake_pedal == on)
    {
      cruise_control=off;
      target_velocity=0;
      setLed(DE2_PIO_GREENLED9_BASE, LED_GREEN_2, 0);
    }
    // reset the position to the beginning of the track
    if(position > 2400)
      position = 0;
    else
      position = position + current_velocity * VEHICLE_PERIOD / 1000;
    show_position(position);
    show_target_velocity(target_velocity);

    err = OSMboxPost(Mbox_Throttle, (void *) &throttle);

    //OSTimeDlyHMSM(0,0,0, CONTROL_PERIOD); //remove for task 4.2
  }
}


void WatchdogTask (void* pdata)
{
  printf ("Watchdog Task created\n");
  INT8U signal = 0;
  INT8U err;
  while (1)
  {
    //initialise semaphore as 0 and run at the end of hyperpeiod
    OSSemPend(Watchdog_Sem, 0 ,&err);

    signal = *((int*)OSMboxPend(Mbox_Watchdog_Signal,1,&err)); //get message
      if (err == OS_NO_ERR)
        {
          printf ("No overload detected\n");
        }
          else
            {
              printf ("CAUTION: OVERLOAD DETECTED\n");
            }
        }
  }

void OverloadTask (void* pdata)
{
  printf ("Overload Task created\n");
  int validation = 1;
  INT8U err;

  while(1)
  {
    OSSemPend(Overload_Sem, 0, &err);
    INT8U err = OSMboxPost(Mbox_Watchdog_Signal,(void*)&validation);
  }
}

void ExtraTask (void* pdata)
{
  printf("Extra Task created \n");

  int utilisation = 0;
  int delay = 0;
  int message = 0;
  INT8U err,err2;
  OS_TMR *DT; //Delay timer for utilisation task

  while (1)
  {
    OSSemPend(Extra_Task_Sem,0,&err);

    int switch_values = switches_pressed();

    if (switch_values & EXTRA_TASK_FLAG)
    {
      utilisation = (switch_values &EXTRA_TASK_FLAG)>>4; //shift back to get value
    }
    else 
    {
      utilisation = 0;
    }

    //50 is the max value but 6 bits have a maximum of 64
    if (utilisation > 49)
    {
      utilisation = 49;
    }
    else
    {
      utilisation = utilisation;
    }

    printf("Utilisation value: %d\n", utilisation);
    delay = MS_IN_TICKS((utilisation+1) * (HYPERPERIOD/50));
    printf("Delay: %d\n",delay);
    //Create timer to cause delay. One-Shot so that it stops after time-out
    DT = OSTmrCreate(delay,0,OS_TMR_OPT_ONE_SHOT,(void*)0,(void*)0,"Delay Timer",&err2);
    OSTmrStart(DT,&err);

    //Create while loop, condition is true while there is some time remaining in timer
    while(OSTmrRemainGet(DT,(void*)&err)>0); //Task doesn't progress while true

    OSTmrStop(DT,OS_TMR_OPT_NONE,(void*)0,&err); //Stop timer
    OSTmrDel(DT,&err2);
  }


}
/* 
 * The task 'StartTask' creates all other tasks kernel objects and
 * deletes itself afterwards.
 */ 
void StartTask(void* pdata)
{
  INT8U err;
  void* context;
  
  static alt_alarm alarm;     /* Is needed for timer ISR function */

  /* Base resolution for SW timer : HW_TIMER_PERIOD ms */
  delay = alt_ticks_per_second() * HW_TIMER_PERIOD / 1000; 
  printf("delay in ticks %d\n", delay);

  /* 
   * Create Hardware Timer with a period of 'delay' 
   */
  if (alt_alarm_start (&alarm,
        delay,
        alarm_handler,
        context) < 0)
  {
    printf("No system clock available!n");
  }

  /* 
   * Create and start Software Timer 
   */
  //start 4.2

  Vehicle_Sem = OSSemCreate(1);
  Control_Sem = OSSemCreate(1);
  ButtonIO_Sem = OSSemCreate(1);
  SwitchIO_Sem = OSSemCreate(1);
  //for 4.5
  Watchdog_Sem = OSSemCreate(0); // should run at hyperperiod
  Overload_Sem = OSSemCreate(1);
  Extra_Task_Sem = OSSemCreate(1);

    Vehicle_Timer = OSTmrCreate(0,
      MS_IN_TICKS(VEHICLE_PERIOD),
      OS_TMR_OPT_PERIODIC,
      Vehicle_Timer_Callback,
      (void *)0,
      "Vehicle Task Timer",&err);

    Control_Timer = OSTmrCreate(0,
      MS_IN_TICKS(CONTROL_PERIOD),
      OS_TMR_OPT_PERIODIC,
      Control_Timer_Callback,
      (void *)0,
      "Control Task Timer",&err);

    ButtonIO_Timer = OSTmrCreate(0,
      MS_IN_TICKS(BUTTON_PERIOD),
      OS_TMR_OPT_PERIODIC,
      ButtonIO_Timer_Callback,
      (void *)0,
      "ButtonIO Task Timer",&err);

    SwitchIO_Timer = OSTmrCreate(0,
      MS_IN_TICKS(SWITCH_PERIOD),
      OS_TMR_OPT_PERIODIC,
      SwitchIO_Timer_Callback,
      (void *)0,
      "Switch Task Timer",&err);

      //TASK 4.5
    Watchdog_Timer = OSTmrCreate(0,
      MS_IN_TICKS(HYPERPERIOD),
      OS_TMR_OPT_PERIODIC,
      Watchdog_Timer_Callback,
      (void *)0,
      "Dog Task Timer",
      &err);

    Overload_Timer = OSTmrCreate(0,
      MS_IN_TICKS(HYPERPERIOD),
      OS_TMR_OPT_PERIODIC,
      Overload_Timer_Callback,
      (void *)0,
      "Overload Task Timer",
      &err);

    Extra_Task_Timer = OSTmrCreate(0,
       MS_IN_TICKS(EXTRA_PERIOD),
      OS_TMR_OPT_PERIODIC,
      Extra_Task_Timer_Callback,
      (void *)0,
      "Extra Task Timer",
      &err);


  // start the new timers
      OSTmrStart(Vehicle_Timer,&err);
      OSTmrStart(Control_Timer,&err);
      OSTmrStart(ButtonIO_Timer,&err);
      OSTmrStart(SwitchIO_Timer,&err);
      //TASK 4.5
      OSTmrStart(Watchdog_Timer, &err);
      OSTmrStart(Overload_Timer,&err);
      OSTmrStart(Extra_Task_Timer,&err);


      printf("error %d\n",err);

  /*
   * Creation of Kernel Objects
   */

  // Mailboxes
  Mbox_Throttle = OSMboxCreate((void*) 0); /* Empty Mailbox - Throttle */
  Mbox_Velocity = OSMboxCreate((void*) 0); /* Empty Mailbox - Velocity */
  Mbox_Brake = OSMboxCreate((void*) 1); /* Empty Mailbox - Velocity */
  Mbox_Engine = OSMboxCreate((void*) 1); /* Empty Mailbox - Engine */
  
  Mbox_EngineControl = OSMboxCreate((void*)1); 
  Mbox_BrakeControl = OSMboxCreate((void*) NULL); /*Empty Mailbox - Brake*/
  Mbox_Gear = OSMboxCreate((void*) 1); /* Empty Mailbox - Gear */
  Mbox_Gas_Pedal = OSMboxCreate((void*) 1); /* Empty Mailbox - Gear */
  Mbox_Cruise_Control = OSMboxCreate((void*) 1); /* Empty Mailbox - Gear */

  //TASK 4.5
  int test=1;
  Mbox_Watchdog_Signal = OSMboxCreate((void*) 1); // to send switch/ overload data  
  Mbox_ExtraTask_Utilization = OSMboxCreate((void*) &test);
  
  /*
   * Create statistics task
   */

  OSStatInit();

  /* 
   * Creating Tasks in the system 
   */


  err = OSTaskCreateExt(
      ControlTask, // Pointer to task code
      NULL,        // Pointer to argument that is
      // passed to task
      &ControlTask_Stack[TASK_STACKSIZE-1], // Pointer to top
      // of task stack
      CONTROLTASK_PRIO,
      CONTROLTASK_PRIO,
      (void *)&ControlTask_Stack[0],
      TASK_STACKSIZE,
      (void *) 0,
      OS_TASK_OPT_STK_CHK);

  err = OSTaskCreateExt(
      VehicleTask, // Pointer to task code
      NULL,        // Pointer to argument that is
      // passed to task
      &VehicleTask_Stack[TASK_STACKSIZE-1], // Pointer to top
      // of task stack
      VEHICLETASK_PRIO,
      VEHICLETASK_PRIO,
      (void *)&VehicleTask_Stack[0],
      TASK_STACKSIZE,
      (void *) 0,
      OS_TASK_OPT_STK_CHK);

  err = OSTaskCreateExt(
       ButtonIO,    // Pointer to task code
       NULL,        // Pointer to argument that is
                    // passed to task
       &ButtonIO_Stack[TASK_STACKSIZE-1], // Pointer to top
                             // of task stack
       BUTTONIO_PRIO,
       BUTTONIO_PRIO,
       (void *)&ButtonIO_Stack[0],
       TASK_STACKSIZE,
       (void *) 0,
       OS_TASK_OPT_STK_CHK);

  err = OSTaskCreateExt(
       SwitchIO,    // Pointer to task code
       NULL,        // Pointer to argument that is
                    // passed to task
       &SwitchIO_Stack[TASK_STACKSIZE-1], // Pointer to top
                             // of task stack
       SWITCHIO_PRIO,
       SWITCHIO_PRIO,
       (void *)&SwitchIO_Stack[0],
       TASK_STACKSIZE,
       (void *) 0,
       OS_TASK_OPT_STK_CHK);

    //TASK 4.5
  err = OSTaskCreateExt(
      WatchdogTask, // Pointer to task code
      NULL,        // Pointer to argument that is
      // passed to task
      &WatchdogTask_Stack[TASK_STACKSIZE-1], // Pointer to top
      // of task stack
      WATCHDOGTASK_PRIO,
      WATCHDOGTASK_PRIO,
      (void *)&WatchdogTask_Stack[0],
      TASK_STACKSIZE,
      (void *) 0,
      OS_TASK_OPT_STK_CHK);

  err = OSTaskCreateExt(
      OverloadTask, // Pointer to task code
      NULL,        // Pointer to argument that is
      // passed to task
      &OverloadTask_Stack[TASK_STACKSIZE-1], // Pointer to top
      // of task stack
      OVERLOADTASK_PRIO,
      OVERLOADTASK_PRIO,
      (void *)&OverloadTask_Stack[0],
      TASK_STACKSIZE,
      (void *) 0,
      OS_TASK_OPT_STK_CHK); 

    err = OSTaskCreateExt(
      ExtraTask, // Pointer to task code
      NULL,        // Pointer to argument that is
      // passed to task
      &ExtraTask_Stack[TASK_STACKSIZE-1], // Pointer to top
      // of task stack
      EXTRATASK_PRIO,
      EXTRATASK_PRIO,
      (void *)&ExtraTask_Stack[0],
      TASK_STACKSIZE,
      (void *) 0,
      OS_TASK_OPT_STK_CHK);
  printf("All Tasks and Kernel Objects generated!\n");

  /* Task deletes itself */

  OSTaskDel(OS_PRIO_SELF);
}

/*
 *
 * The function 'main' creates only a single task 'StartTask' and starts
 * the OS. All other tasks are started from the task 'StartTask'.
 *
 */

int main(void) {
  printf("Lab: Cruise Control 20216755765\n");

  OSTaskCreateExt(
      StartTask, // Pointer to task code
      NULL,      // Pointer to argument that is
      // passed to task
      (void *)&StartTask_Stack[TASK_STACKSIZE-1], // Pointer to top
      // of task stack 
      STARTTASK_PRIO,
      STARTTASK_PRIO,
      (void *)&StartTask_Stack[0],
      TASK_STACKSIZE,
      (void *) 0,  
      OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);

  OSStart();

  return 0;
}