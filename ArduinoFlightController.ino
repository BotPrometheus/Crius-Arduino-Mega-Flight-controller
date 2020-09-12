#include <LiquidCrystal_I2C.h>
#include "PinChangeInterrupt.h"
#include <Wire.h>                       //Include the Wire.h library so we can communicate with the gyro.

LiquidCrystal_I2C lcd(0x27,16,2);       //Initialize the LCD library

unsigned long TimerCh[7];           //Η ώρα σε microsecond για τον υπολογισμό PWM καναλιών
unsigned long TimerSC[5];           //Η ώρα σε microsecond που θα σταματίσει ο παλμός στο speedControler
unsigned long TimerLoop;            //Η ώρα κύκλου
int TimeLenghtPWMChan[7];           //Χρονικό διάστημα παλμού PWM τηλεκατεύθυνση

int temperature;                            //Η τιμή θερμοκρασίας από το MPU6050
long acc_x, acc_y, acc_z, acc_total_vector; //Οι γραμμικές επιταχύνσεις από το MPU6050 κατά x-y-z. Όχι μετακινήσεις.
double gyro_pitch, gyro_roll, gyro_yaw;     //Οι γωνιακές επιταχύνσεις από το MPU6050 κατά x-y-z. Όχι γωνίες.
long gyro_axis_cal[4];          //Πίνακας αποθήκευσης τιμών απο αρχικές μετρήσεις γυροσκοπίου στο setup οπου προκύπει ο μέσος όρος διόρθωσης γωνίας για την γωνία που ξεκινάει το drone

int esc_1, esc_2, esc_3, esc_4; //Οι τιμές των speed controller που προκύπτουν από υπολογισμούς

int numOfCalibrations;//Αριθμός αρχικών μετρήσεων από το γυροσκόπιο MPU6050 για να υπολογιστή ο διορθωτικός πίνακας gyro_axis_cal[]. Αριθμός διορθώσεων = 2000.
int start;  //Κατάσταση drone 0=ανενεργο, 1=yaw αριστερά και throttle κάτω αναμένει να πάει το yaw στο κέντρο, 2=yaw κέντρο και throttle κάτω έτοιμο να πετάξει
float angle_roll_acc, angle_pitch_acc;  //Υπολογισμός roll - pitch από το επιταχυνσιόμετρο
float angle_pitch, angle_roll;          //Τελικές τιμές roll - pitch από συνδυασμό επιταχυνσιόμετρου και γυροσκοποίου
boolean auto_level = true;              //Auto level on (true) or off (false)
int throttle, battery_voltage;          //Μεταβλητή που δέχεται το γκάζι από την τηλεκατεύθυνση, Μεταβλητή που δείχνει την τάση της μπαταρίας

#define interrupt_Ch1_Roll     A9     //CHANNEL 1 ==> A9  Roll
#define interrupt_Ch2_Pitch    A10    //CHANNEL 2 ==> A10 Pitch
#define interrupt_Ch3_Throttle A8     //CHANNEL 3 ==> A8  Throttle
#define interrupt_Ch4_Yaw      A11    //CHANNEL 4 ==> A11 Yaw
#define interrupt_Ch5_AUX1     A12    //CHANNEL 5 ==> A12 AUX1
#define interrupt_Ch6_AUX2     A13    //CHANNEL 6 ==> A13 AUX2

#define LED_RED_A    13					//Red led of the board
#define LED_ORANGE_B 31					//Orange led of the board
#define LED_GREEN_C  30					//Green led of the board

float roll_level_adjust, pitch_level_adjust;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_roll,  pid_roll_setpoint,  gyro_roll_input,  pid_output_roll,  pid_last_roll_d_error;
float pid_error_temp;

float pid_p_gain_roll = 1.3;               //Gain setting for the roll P-controller
float pid_i_gain_roll = 0.04;              //Gain setting for the roll I-controller
float pid_d_gain_roll = 18.0;              //Gain setting for the roll D-controller
int   pid_max_roll    = 400;               //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int   pid_max_pitch    = pid_max_roll;     //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 4.0;                //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0.02;               //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller.
int   pid_max_yaw    = 400;                //Maximum output of the PID-controller (+/-)

float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;


double Ch1A, Ch2A, Ch3A, Ch4A, Ch5A, Ch6A, Ch7A, Ch8A, Ch9A;     //Μεταβλητές που περιέχουν το 1000/(μέγιστο-ελάχιστο) που εκπέμπει το κάθε κανάλι
int channel1TrData[2];
int channel2TrData[2];
int channel3TrData[2];
int channel4TrData[2];
int channel5TrData[2];
int channel6TrData[2];
int channel7TrData[2];
int channel8TrData[2];
int channel9TrData[2];

int ChannelsMinValue;int ChannelsMaxValue;      //Το ελάχιστο και μέγιστο που θέλω να προκύπτει από κάθε κανάλι μετά από γραμμική παρεμβολή
int ChannelMedValue[3]; //Το μέσο που θέλω να προκύπτει από κάθε κανάλι μετά από γραμμική παρεμβολή. Το 2ο του πίνακα είναι το μέσο,το 1ο το κάτω όριο του μέσου και το 3ο το πάνω όριο του μέσου
volatile int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4, receiver_input_channel_5, receiver_input_channel_6;

//Βοηθητικές μεταβλητές για debugging
unsigned long loop_timer_LCD=0;

void setup(){
    Serial.begin(115200);             //Use only for debugging
    pinMode(LED_RED_A, OUTPUT);      	//Το κόκκινο LED της κάρτας γίνεται διαθέσιμο
    pinMode(LED_GREEN_C, OUTPUT);       //Το πράσινο LED της κάρτας γίνεται διαθέσιμο
    pinMode(LED_ORANGE_B, OUTPUT);      //Το πορτοκαλί LED της κάρτας γίνεται διαθέσιμο
    digitalWrite(LED_RED_A, false);digitalWrite(LED_GREEN_C, false);digitalWrite(LED_ORANGE_B, false);
    //Σύνδεση interrupt pin με τις συναρτήσεις των καναλιών
    attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(interrupt_Ch1_Roll),     Ch1_Roll,     CHANGE);
    attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(interrupt_Ch2_Pitch),    Ch2_Pitch,    CHANGE);
    attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(interrupt_Ch3_Throttle), Ch3_Throttle, CHANGE);
    attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(interrupt_Ch4_Yaw),      Ch4_Yaw,      CHANGE);
    attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(interrupt_Ch5_AUX1),     Ch5_AUX1,     CHANGE);
    attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(interrupt_Ch6_AUX2),     Ch6_AUX2,     CHANGE);

    setupChannelCalibrationValues();

    Wire.begin();                     //Start the I2C as master.
    start = 0;                        //Set start to zero.
    TWBR = 12;                        //Set the I2C clock speed to 400kHz. TWBR = 0.5 * (F_CPU / [ταχύτητα Ι2C] - 16), F_CPU=[ταχύτητα επεξεργαστή arduino]=16x10^6 , [ταχύτητα Ι2C] = 400kh για το γυροσκόπιο, Επομένος 0,5*(16*10^6/(400000)-16) = 12

    //Arduino (Atmega) pins default to inputs, so they don't need to be explicitly declared as inputs.
    //Configure digital pin 2, 3, 5 and 6 as output. Τα pin της κάρτας που θα τοποθετηθούν τα speed controllers
    DDRE |= B00111000;          //Τα D2=S2=BIT_4, D3=S3=BIT_5, D5=S5=BIT_3, της κάρτας που θα τοποθετηθούν τα speed controllers
    DDRH |= B00001000;          //Τα D6=S6=BIT_4

    set_gyro_registers();             //Set the specific gyro registers.

    lcd.init();                       //Initialize the LCD
    lcd.noBacklight();                //DeActivate backlight
    lcd.clear();                      //Clear the LCD

    //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
    for (numOfCalibrations = 0; numOfCalibrations < 2000 ; numOfCalibrations ++)                            //Take 2000 readings for calibration.
    {
        if(numOfCalibrations % 15 == 0)
            digitalWrite(LED_RED_A, !digitalRead(LED_RED_A));               //Change the led status to indicate calibration.

        gyro_signalen();                                                    //Read the gyro output.
        gyro_axis_cal[1] += gyro_pitch;                                     //Ad roll value to gyro_roll_cal.
        gyro_axis_cal[2] += gyro_roll;                                      //Ad pitch value to gyro_pitch_cal.
        gyro_axis_cal[3] += gyro_yaw;                                       //Ad yaw value to gyro_yaw_cal.
        //We don't want the esc's to be beeping annoyingly. So let's give them a 1000us puls while calibrating the gyro.
        PORTE |= B00111000;PORTH |= B00001000;                              //Set digital poort 2, 3, 5 and 6 high.
        delayMicroseconds(1000);                                            //Wait 1000us.
        PORTE = B00000000;PORTH = B00000000;                                //Set digital poort 2, 3, 5 and 6 low.
        delayMicroseconds(3000);                                            //Wait 3 milliseconds before the next loop.
    }

    //Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset.
    gyro_axis_cal[1] /= 2000;                                                 //Divide the roll total by 2000.
    gyro_axis_cal[2] /= 2000;                                                 //Divide the pitch total by 2000.
    gyro_axis_cal[3] /= 2000;                                                 //Divide the yaw total by 2000.


    //Wait until the throtle is set to the lower position.
    do{
		receiver_input_channel_3 = convert_receiver_channel(3);
		start ++;                                                       //While waiting increment start whith every loop.
		//We don't want the esc's to be beeping annoyingly. So let's give them a 1000us puls while waiting for the receiver inputs.
		PORTE |= B00111000;PORTH |= B00001000;                          //Set digital poort 2, 3, 5 and 6 high.
		delayMicroseconds(1000);                                        //Wait 1000us.
		PORTE &= B11000111;PORTH &= B11110111;                          //Set digital poort 2, 3, 5 and 6 low.
		delayMicroseconds(3000);                                      //Wait 3000us before the next loop.
		if(start == 125)
		{                                               				//Every 125 loops (500ms).
			digitalWrite(LED_ORANGE_B, !digitalRead(LED_ORANGE_B));       //Change the led status.
			start = 0;                                                    //Start again at 0.
		}
	}while(receiver_input_channel_3 > 1020);
	start = 0;

    /*Load the battery voltage to the battery_voltage variable.
    65 is the voltage compensation for the diode.
    12.6V equals ~5V @ Analog 0.
    12.6V equals 1023 analogRead(0).
    1260 / 1023 = 1.2317.
    The variable battery_voltage holds 1050 if the battery voltage is 10.5V.*/
    battery_voltage = (analogRead(0) + 65) * 1.2317;
    //When everything is done, turn off the led.
    digitalWrite(LED_ORANGE_B, false);                                          //Turn off the warning led.
    TimerLoop = micros();                                                    	//Set the timer for the next loop.
}

void loop() {
    if(millis()-loop_timer_LCD >= 1000){
        loop_timer_LCD = millis();
          //Serial.println(loop_timer_LCD);
          //Serial.print("ch1:      ");Serial.print(TimeLenghtPWMChan[1]);Serial.print("===>");Serial.println(receiver_input_channel_1);
          //Serial.print("ch2 Pitch ");Serial.print(TimeLenghtPWMChan[2]);Serial.print("===>");Serial.println(receiver_input_channel_2);
          //Serial.print("ch3 Throttle ");Serial.print(TimeLenghtPWMChan[3]);Serial.print("===>");Serial.println(receiver_input_channel_3);
          //Serial.print("ch4 Yaw      ");Serial.print(TimeLenghtPWMChan[4]);Serial.print("===>");Serial.println(receiver_input_channel_4);
          //Serial.print("ch5 AUX1     ");Serial.print(TimeLenghtPWMChan[5]);Serial.print("===>");Serial.println(receiver_input_channel_5);
          //Serial.print("ch6 AUX2     ");Serial.print(TimeLenghtPWMChan[6]);Serial.print("===>");Serial.println(receiver_input_channel_6);
          //Serial.print("throttle     ");Serial.println(throttle);
          //Serial.print("esc_1        ");Serial.println(esc_1);
          //Serial.print("esc_2        ");Serial.println(esc_2);
          //Serial.print("esc_3        ");Serial.println(esc_3);
          //Serial.print("esc_4        ");Serial.println(esc_4);
          Serial.print("P ");Serial.println(angle_pitch);
          Serial.print("R ");Serial.println(angle_roll);
          //Serial.print("S ");Serial.println(start);
    }
    //65.5 = 1 deg/sec (check the datasheet of the MPU-6050 for more information).
    gyro_roll_input  = (gyro_roll_input  * 0.7) + ((gyro_roll  / 65.5) * 0.3); //Gyro pid input is deg/sec.
    gyro_pitch_input = (gyro_pitch_input * 0.7) + ((gyro_pitch / 65.5) * 0.3); //Gyro pid input is deg/sec.
    gyro_yaw_input   = (gyro_yaw_input   * 0.7) + ((gyro_yaw   / 65.5) * 0.3); //Gyro pid input is deg/sec.
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    //This is the added IMU code from the videos:
    //https://youtu.be/4BoIE8YQwM8
    //https://youtu.be/j-kE0AMEWy4
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    //Gyro angle calculations
    //0.0000611 = 1 / (250Hz / 65.5)
    //Πολλαπλασιάζω επί 2 γιατί αντι για περίοδο 1/250=4ms χρησιμοποιώ περίοδο 1/125=8ms για να προλαβαίνει το arduino να διαβάσει το γυροσκόπιο και μελλοντικούς αισθητήρες
    angle_pitch += 2*gyro_pitch * 0.0000611;      //Calculate the traveled pitch angle and add this to the angle_pitch variable.
    angle_roll  += 2*gyro_roll  * 0.0000611;      //Calculate the traveled roll angle and add this to the angle_roll variable.

    //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
    //Πολλαπλασιάζω επί 2 γιατί αντι για περίοδο 1/250=4ms χρησιμοποιώ περίοδο 1/125=8ms για να προλαβαίνει το arduino να διαβάσει το γυροσκόπιο και μελλοντικούς αισθητήρες
    angle_pitch -= angle_roll  * sin(2*gyro_yaw * 0.000001066);                  //If the IMU has yawed transfer the roll angle to the pitch angel.
    angle_roll  += angle_pitch * sin(2*gyro_yaw * 0.000001066);                  //If the IMU has yawed transfer the pitch angle to the roll angel.
    //Accelerometer angle calculations
    acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));       //Calculate the total accelerometer vector.

    if(abs(acc_y) < acc_total_vector){                                        //Prevent the asin function to produce a NaN
      angle_pitch_acc = asin((float)acc_y/acc_total_vector)*  57.296;          //Calculate the pitch angle.
    }
    if(abs(acc_x) < acc_total_vector){                                        //Prevent the asin function to produce a NaN
      angle_roll_acc  = asin((float)acc_x/acc_total_vector)* -57.296;          //Calculate the roll angle.
    }

    //Place the MPU-6050 spirit level and note the values in the following two lines for calibration.
    angle_pitch_acc -= 0.0;                                                   //Accelerometer calibration value for pitch.
    angle_roll_acc  -= 0.0;                                                    //Accelerometer calibration value for roll.

    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;            //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
    angle_roll  = angle_roll  * 0.9996 + angle_roll_acc  * 0.0004;            //Correct the drift of the gyro roll angle with the accelerometer roll angle.

    pitch_level_adjust = angle_pitch * 15;                                    //Calculate the pitch angle correction
    roll_level_adjust  = angle_roll  * 15;                                    //Calculate the roll angle correction

    if(!auto_level){                                                          //If the quadcopter is not in auto-level mode
      pitch_level_adjust = 0;                                                 //Set the pitch angle correction to zero.
      roll_level_adjust = 0;                                                  //Set the roll angle correcion to zero.
    }
    //For starting the motors: throttle low and yaw left (step 1).
    if(receiver_input_channel_3 < 1050 && receiver_input_channel_4 < 1050){
        start = 1;
        digitalWrite(LED_GREEN_C, true);
    }
    //When yaw stick is back in the center position start the motors (step 2).
    if(start == 1 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1450){
      start = 2;
      angle_pitch = angle_pitch_acc;                                          //Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
      angle_roll = angle_roll_acc;                                            //Set the gyro roll angle equal to the accelerometer roll angle when the quadcopter is started.
      //Reset the PID controllers for a bumpless start.
      pid_i_mem_roll = 0;
      pid_last_roll_d_error = 0;
      pid_i_mem_pitch = 0;
      pid_last_pitch_d_error = 0;
      pid_i_mem_yaw = 0;
      pid_last_yaw_d_error = 0;
    }
    //Stopping the motors: throttle low and yaw right.
    if(start == 2 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1950){
        start = 0;
        digitalWrite(LED_ORANGE_B, false);
        digitalWrite(LED_GREEN_C, false);
    }
    //The PID set point in degrees per second is determined by the roll receiver input.
    //In the case of deviding by 3 the max roll rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
    pid_roll_setpoint = 0;
    //We need a little dead band of 16us for better results.
    if(receiver_input_channel_1 > 1508)      pid_roll_setpoint = receiver_input_channel_1 - 1508;
    else if(receiver_input_channel_1 < 1492) pid_roll_setpoint = receiver_input_channel_1 - 1492;
    pid_roll_setpoint -= roll_level_adjust;//Subtract the angle correction from the standardized receiver roll input value.
    pid_roll_setpoint /= 3.0;              //Divide the setpoint for the PID roll controller by 3 to get angles in degrees.

    //The PID set point in degrees per second is determined by the pitch receiver input.
    //In the case of deviding by 3 the max pitch rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
    pid_pitch_setpoint = 0;
    //We need a little dead band of 16us for better results.
    if     (receiver_input_channel_2 > 1508)pid_pitch_setpoint = receiver_input_channel_2 - 1508;
    else if(receiver_input_channel_2 < 1492)pid_pitch_setpoint = receiver_input_channel_2 - 1492;
    pid_pitch_setpoint -= pitch_level_adjust;     //Subtract the angle correction from the standardized receiver pitch input value.
    pid_pitch_setpoint /= 3.0;                    //Divide the setpoint for the PID pitch controller by 3 to get angles in degrees.

    //The PID set point in degrees per second is determined by the yaw receiver input.
    //In the case of deviding by 3 the max yaw rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
    pid_yaw_setpoint = 0;
    //We need a little dead band of 16us for better results.
    if(receiver_input_channel_3 > 1050){ //Do not yaw when turning off the motors.
      if(receiver_input_channel_4 > 1508)      pid_yaw_setpoint = (receiver_input_channel_4 - 1508)/3.0;
      else if(receiver_input_channel_4 < 1492) pid_yaw_setpoint = (receiver_input_channel_4 - 1492)/3.0;
    }
    calculate_pid();              //PID inputs are known. So we can calculate the pid output.
    //The battery voltage is needed for compensation.
    //A complementary filter is used to reduce noise.
    //0.09853 = 0.08 * 1.2317.
    battery_voltage = battery_voltage * 0.92 + (analogRead(0) + 65) * 0.09853;
    //Turn on the led if battery voltage is to low.
    if(battery_voltage < 1000 && battery_voltage > 600)   digitalWrite(LED_RED_A, HIGH);
    throttle = receiver_input_channel_3;                                      //We need the throttle signal as a base signal.

    if (start == 2){                                                          //The motors are started.
      if (throttle > 1500) throttle = 1500;                                   //We need some room to keep full control at full throttle.
      esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW)
      esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW)
      esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW)
      esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW)

      /*if (battery_voltage < 1240 && battery_voltage > 800){                   //Is the battery connected?
	        esc_1 += esc_1 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-1 pulse for voltage drop.
	        esc_2 += esc_2 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-2 pulse for voltage drop.
	        esc_3 += esc_3 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-3 pulse for voltage drop.
	        esc_4 += esc_4 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-4 pulse for voltage drop.
    	}*/

      if (esc_1 < 1100) esc_1 = 1100;                                         //Keep the motors running.
      if (esc_2 < 1100) esc_2 = 1100;                                         //Keep the motors running.
      if (esc_3 < 1100) esc_3 = 1100;                                         //Keep the motors running.
      if (esc_4 < 1100) esc_4 = 1100;                                         //Keep the motors running.

      if (esc_1 > 1600) esc_1 = 1600;                                           //Limit the esc-1 pulse to 2000us.
      if (esc_2 > 1600) esc_2 = 1600;                                           //Limit the esc-2 pulse to 2000us.
      if (esc_3 > 1600) esc_3 = 1600;                                           //Limit the esc-3 pulse to 2000us.
      if (esc_4 > 1600) esc_4 = 1600;                                           //Limit the esc-4 pulse to 2000us.
    }

    else{
      esc_1 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-1.
      esc_2 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-2.
      esc_3 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-3.
      esc_4 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-4.
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////
    //Creating the pulses for the ESC's is explained in this video:
    //https://youtu.be/fqEkVcqxtU8
    ////////////////////////////////////////////////////////////////////////////////////////////////////

    //! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !
    //Because of the angle calculation the loop time is getting very important. If the loop time is
    //longer or shorter than 4000us(tas 8000us) the angle calculation is off. If you modify the code make sure
    //that the loop time is still 4000us and no longer! More information can be found on
    //the Q&A page:
    //! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !
    if(micros() - TimerLoop > 8050)   digitalWrite(LED_ORANGE_B, HIGH);        //Turn on the LED if the loop time exceeds 4050us.(tas 8050us)
    //All the information for controlling the motor's is available.
    //The refresh rate is 250Hz(tas 125Hz). That means the esc's need there pulse every 8ms.
    while(micros() - TimerLoop < 8000);                                 //We wait until 8000us are passed.
    TimerLoop = micros();                                               //Set the timer for the next loop.
    PORTE |= B00111000;PORTH |= B00001000;                              //Set digital outputs 2,3,5 and 6 high.
    TimerSC[1] = esc_1 + TimerLoop;                                     //Calculate the time of the faling edge of the esc-1 pulse.
    TimerSC[2] = esc_2 + TimerLoop;                                     //Calculate the time of the faling edge of the esc-2 pulse.
    TimerSC[3] = esc_3 + TimerLoop;                                     //Calculate the time of the faling edge of the esc-3 pulse.
    TimerSC[4] = esc_4 + TimerLoop;                                     //Calculate the time of the faling edge of the esc-4 pulse.
    //Κενός χρόνος 1000us οπού μπορεόυμε να εκμεταλευτούμε για να διαβάσουμε τιμές αισθητήρων. Ο χρόνος ανάγνωσης δεν πρέπει να ξεπεράσει ποτέ 1000us
    while(PORTE >= 8 || PORTH >=8){                                           //Stay in this loop until output 2,3,5 and 6 are low.
      unsigned long   esc_loop_timer = micros();                                              //Read the current time.
      if(TimerSC[1] <= esc_loop_timer)PORTE &= B11101111;                //Set digital output 2 [4] to low if the time is expired. Το D2 είναι το 5ο bit PORTE
      if(TimerSC[2] <= esc_loop_timer)PORTE &= B11011111;                //Set digital output 3 [5] to low if the time is expired. Το D3 είναι το 6ο bit PORTE
      if(TimerSC[3] <= esc_loop_timer)PORTE &= B11110111;                //Set digital output 5 [6] to low if the time is expired. Το D5 είναι το 4ο bit PORTE
      if(TimerSC[4] <= esc_loop_timer)PORTH &= B11110111;                //Set digital output 6 [7] to low if the time is expired. Το D6 είναι το 4ο bit PORTH
    }
    //Συναρτήσεις που θα διαχειρίζονται μετρήσεις αισθητήρων εδώ
    gyro_signalen();
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Subroutine for reading the gyro
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void gyro_signalen(){
    //Read the MPU-6050
    Wire.beginTransmission(0x68);                 //Start communication with the gyro.
    Wire.write(0x3B);                             //Start reading @ register 43h and auto increment with every read.
    Wire.endTransmission();                       //End the transmission.
    Wire.requestFrom(0x68,14);                    //Request 14 bytes from the gyro.

    receiver_input_channel_1 = convert_receiver_channel(1);                 //Convert the actual receiver signals for pitch to the standard 1000 - 2000us.
    receiver_input_channel_2 = convert_receiver_channel(2);                 //Convert the actual receiver signals for roll to the standard 1000 - 2000us.
    receiver_input_channel_3 = convert_receiver_channel(3);                 //Convert the actual receiver signals for throttle to the standard 1000 - 2000us.
    receiver_input_channel_4 = convert_receiver_channel(4);                 //Convert the actual receiver signals for yaw to the standard 1000 - 2000us.
    receiver_input_channel_5 = convert_receiver_channel(5);
    receiver_input_channel_6 = convert_receiver_channel(6);

    while(Wire.available() < 14);                                           //Wait until the 14 bytes are received.
    acc_x        = Wire.read()<<8|Wire.read();                              //Add the low and high byte to the acc_x variable.
    acc_y        = Wire.read()<<8|Wire.read();                              //Add the low and high byte to the acc_y variable.
    acc_z        = Wire.read()<<8|Wire.read();                              //Add the low and high byte to the acc_z variable.
    temperature  = Wire.read()<<8|Wire.read();                              //Add the low and high byte to the temperature variable.
    gyro_pitch   = Wire.read()<<8|Wire.read();                              //Read high and low part of the angular data.
    gyro_roll    = Wire.read()<<8|Wire.read();                              //Read high and low part of the angular data.
    gyro_yaw     = Wire.read()<<8|Wire.read();                              //Read high and low part of the angular data.

    if(numOfCalibrations == 2000){
        gyro_pitch -= gyro_axis_cal[1];                                   //Only compensate after the calibration. X
        gyro_roll  -= gyro_axis_cal[2];                                   //Only compensate after the calibration. Y
        gyro_yaw   -= gyro_axis_cal[3];                                   //Only compensate after the calibration. Z
    }
}

void set_gyro_registers(){
  //Setup the MPU-6050
  //Activate the MPU-6050
    Wire.beginTransmission(0x68);                                              //Start communication with the address found during search.
    Wire.write(0x6B);                                                          //We want to write to the PWR_MGMT_1 register (6B hex)
    Wire.write(0x00);                                                          //Set the register bits as 00000000 to activate the gyro
    Wire.endTransmission();                                                    //End the transmission with the gyro.
    //Configure the gyro (500dps full scale)
    Wire.beginTransmission(0x68);                                      //Start communication with the address found during search.
    Wire.write(0x1B);                                                          //We want to write to the GYRO_CONFIG register (1B hex)
    Wire.write(0x08);                                                          //Set the register bits as 00001000 (500dps full scale)
    Wire.endTransmission();                                                    //End the transmission with the gyro
    //Configure the accelerometer (+/-8g)
    Wire.beginTransmission(0x68);                                      //Start communication with the address found during search.
    Wire.write(0x1C);                                                          //We want to write to the ACCEL_CONFIG register (1A hex)
    Wire.write(0x10);                                                          //Set the register bits as 00010000 (+/- 8g full scale range)
    Wire.endTransmission();                                                    //End the transmission with the gyro
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Subroutine for calculating pid outputs
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//The PID controllers are explained in part 5 of the YMFC-3D video session:
//https://youtu.be/JBvnB0279-Q
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void calculate_pid(){
  //Roll calculations
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if     (pid_i_mem_roll > pid_max_roll     )  pid_i_mem_roll = pid_max_roll;
  else if(pid_i_mem_roll < pid_max_roll * -1)  pid_i_mem_roll = pid_max_roll * -1;

  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if(pid_output_roll > pid_max_roll)  pid_output_roll = pid_max_roll;
  else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;

  pid_last_roll_d_error = pid_error_temp;

  //Pitch calculations
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;

  pid_last_pitch_d_error = pid_error_temp;

  //Yaw calculations
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
  else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;

  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;

  pid_last_yaw_d_error = pid_error_temp;
}

void setupChannelCalibrationValues(){
    ChannelsMinValue=1000;ChannelsMaxValue=2000;      //Το ελάχιστο και μέγιστο που θέλω να προκύπτει από κάθε κανάλι μετά από γραμμική παρεμβολή

    channel1TrData[0]=1048;channel1TrData[1]=1872;Ch1A=(double)ChannelsMinValue/(channel1TrData[1]-channel1TrData[0]);
    channel2TrData[0]=1040;channel2TrData[1]=1860;Ch2A=(double)ChannelsMinValue/(channel2TrData[1]-channel2TrData[0]);
    channel3TrData[0]=1060;channel3TrData[1]=1864;Ch3A=(double)ChannelsMinValue/(channel3TrData[1]-channel3TrData[0]);
    channel4TrData[0]=1040;channel4TrData[1]=1860;Ch4A=(double)ChannelsMinValue/(channel4TrData[1]-channel4TrData[0]);
    channel5TrData[0]=1004;channel5TrData[1]=1828;Ch5A=(double)ChannelsMinValue/(channel5TrData[1]-channel5TrData[0]);
    channel6TrData[0]=1008;channel6TrData[1]=1828;Ch6A=(double)ChannelsMinValue/(channel6TrData[1]-channel6TrData[0]);
    channel7TrData[0]=0;   channel7TrData[1]=0;   Ch7A=(double)ChannelsMinValue/(channel7TrData[1]-channel7TrData[0]);
    channel8TrData[0]=0;   channel8TrData[1]=0;   Ch8A=(double)ChannelsMinValue/(channel8TrData[1]-channel8TrData[0]);
    channel9TrData[0]=0;   channel9TrData[1]=0;   Ch9A=(double)ChannelsMinValue/(channel9TrData[1]-channel9TrData[0]);

    ChannelMedValue[1]=(ChannelsMaxValue+ChannelsMinValue)/2;
    ChannelMedValue[0]=ChannelMedValue[1]-8;
    ChannelMedValue[2]=ChannelMedValue[1]+8;
    gyro_axis_cal[1]=gyro_axis_cal[2]=gyro_axis_cal[3]=0;
}

void Ch1_Roll(void) {
    uint8_t trigger = getPinChangeInterruptTrigger(digitalPinToPCINT(interrupt_Ch1_Roll));
    if(trigger == RISING){
        TimerCh[1]=micros();
    }
    else if(trigger == FALLING){
        TimeLenghtPWMChan[1] = micros()-TimerCh[1];
    }
}

void Ch2_Pitch(void) {
    uint8_t trigger = getPinChangeInterruptTrigger(digitalPinToPCINT(interrupt_Ch2_Pitch));
    if(trigger == RISING){
        TimerCh[2]=micros();
    }
    else if(trigger == FALLING){
        TimeLenghtPWMChan[2] = micros()-TimerCh[2];
    }
}

void Ch3_Throttle(void) {
    uint8_t trigger = getPinChangeInterruptTrigger(digitalPinToPCINT(interrupt_Ch3_Throttle));
    if(trigger == RISING){
        TimerCh[3]=micros();
    }
    else if(trigger == FALLING){
        TimeLenghtPWMChan[3] = micros()-TimerCh[3];
    }
}

void Ch4_Yaw(void) {
    uint8_t trigger = getPinChangeInterruptTrigger(digitalPinToPCINT(interrupt_Ch4_Yaw));
    if(trigger == RISING){
        TimerCh[4]=micros();
    }
    else if(trigger == FALLING){
        TimeLenghtPWMChan[4] = micros()-TimerCh[4];
    }
}

void Ch5_AUX1(void) {
    uint8_t trigger = getPinChangeInterruptTrigger(digitalPinToPCINT(interrupt_Ch5_AUX1));
    if(trigger == RISING){
        TimerCh[5]=micros();
    }
    else if(trigger == FALLING){
        TimeLenghtPWMChan[5] = micros()-TimerCh[5];
    }
}

void Ch6_AUX2(void) {
    uint8_t trigger = getPinChangeInterruptTrigger(digitalPinToPCINT(interrupt_Ch6_AUX2));
    if(trigger == RISING){
        TimerCh[6]=micros();
    }
    else if(trigger == FALLING){
        TimeLenghtPWMChan[6] = micros()-TimerCh[6];
    }
}

//This part converts the actual receiver signals to a standardized 1000 � 1500 � 2000 microsecond value.
//The stored data in the EEPROM is used.
int convert_receiver_channel(int channel){
    int answer=-1;
    if(channel==1)
    {
        answer = (ChannelsMinValue + ((TimeLenghtPWMChan[1] - channel1TrData[0]) * Ch1A));
        if(answer<ChannelsMinValue)
            answer=ChannelsMinValue;
        else if(answer>ChannelMedValue[0] && answer<ChannelMedValue[2])
            answer=ChannelMedValue[1];
        else if(answer>ChannelsMaxValue)
            answer=ChannelsMaxValue;

        return answer;
    }
    else if(channel==2)
    {
        answer = (ChannelsMinValue + ((TimeLenghtPWMChan[2] - channel2TrData[0]) * Ch2A));
        if(answer<ChannelsMinValue)
            answer=ChannelsMinValue;
        else if(answer>ChannelMedValue[0] && answer<ChannelMedValue[2])
            answer=ChannelMedValue[1];
        else if(answer>ChannelsMaxValue)
            answer=ChannelsMaxValue;

        return answer;
    }
    else if(channel==3)
    {
        answer = (ChannelsMinValue + ((TimeLenghtPWMChan[3] - channel3TrData[0]) * Ch3A));

        if(answer<ChannelsMinValue)
            answer=ChannelsMinValue;
        else if(answer>ChannelMedValue[0] && answer<ChannelMedValue[2])
            answer=ChannelMedValue[1];
        else if(answer>ChannelsMaxValue)
            answer=ChannelsMaxValue;

        return answer;
    }
    else if(channel==4)
    {
        answer = (ChannelsMinValue + ((TimeLenghtPWMChan[4] - channel4TrData[0]) * Ch4A));
        if(answer<ChannelsMinValue)
            answer=ChannelsMinValue;
        else if(answer>ChannelMedValue[0] && answer<ChannelMedValue[2])
            answer=ChannelMedValue[1];
        else if(answer>ChannelsMaxValue)
            answer=ChannelsMaxValue;

        return answer;
    }
    else if(channel==5)
    {
        answer = (ChannelsMinValue + ((TimeLenghtPWMChan[5] - channel5TrData[0]) * Ch5A));
        if(answer<ChannelsMinValue)
            answer=ChannelsMinValue;
        else if(answer>ChannelMedValue[0] && answer<ChannelMedValue[2])
            answer=ChannelMedValue[1];
        else if(answer>ChannelsMaxValue)
            answer=ChannelsMaxValue;

        return answer;
    }
    else if(channel==6)
    {
        answer = (ChannelsMinValue + ((TimeLenghtPWMChan[6] - channel6TrData[0]) * Ch6A));
        if(answer<ChannelsMinValue)
            answer=ChannelsMinValue;
        else if(answer>ChannelMedValue[0] && answer<ChannelMedValue[2])
            answer=ChannelMedValue[1];
        else if(answer>ChannelsMaxValue)
            answer=ChannelsMaxValue;

        return answer;
    }
}
