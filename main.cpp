//Head Coder: Nishant Sathe
//Teammates: Alex King, Michelle Lee, Hrithik Basu
//Team: Press F To Pay Respects (D5)

//Including all libraries
#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHServo.h>
#include <math.h>
#include <LCDColors.h>

//Instantiation of devices
AnalogInputPin photoresis(FEHIO::P0_1);
FEHMotor leftmotor(FEHMotor::Motor0,9.0);//the left motor is on port 0 on the proteus
FEHMotor rightmotor(FEHMotor::Motor1,9.0);//the right motor is on port 1 on the proteus
DigitalEncoder left_encoder(FEHIO::P0_0);//left motor encoder is current set to second port of 0 bank on proteus
DigitalEncoder right_encoder(FEHIO::P0_1);//right motor encoder is currently set to first port of 0 bank on proteus

//Defining constants for use throughout the code
#define FORWARD_PERCENT -25.0//This defines the default speed at which the robot goes forward
#define COUNTS_PER_INCH 33.74//This defines how far per inch the robot will go in specification with the encoder. There are 318 counts in one revolution.
#define BACKWARDS_PERCENT 18.0//This defines how fast the robot will go while moving backwards
#define COUNTS_PER_DEGREE 1.7//This defines how many counts the encoder should do per degree
#define DEBUG 1

//PID constants, tweak for tuning
float Kp = 3;
float Ki = 0.5;
float Kd = 0;

int ddrCheck=0;


void startUp(){//This function waits until the proteus screen has been tapped to start the instruction set

        float x,y;
        LCD.Clear(BLACK);
        LCD.SetFontColor(WHITE);
        LCD.WriteLine("Touch the screen to start");
        while(!LCD.Touch(&x,&y)){} //Wait for screen to be pressed
        while(LCD.Touch(&x,&y)){} //Wait for screen to be unpressed

}

//http://robotsforroboticists.com/pid-control/

void move(float speed, float distance){//direction 1 is forward, direction -1 is backwards, distance in inches, this function tells the robot which direction to move in and how far
    //int counts = abs(distance)*COUNTS_PER_INCH;//the number of counts is equal to the distance times the defined constant COUNTS_PER_INCH
    int counts = distance;      //Used for testing, revert to above line when finished

    float inputSpeed = (speed/100)*225;      //Convert from percentage speed value to a speed in RPMs

    //DEBUG
    LCD.Write("Target speed: ");
    LCD.WriteLine(inputSpeed);
    LCD.WriteLine(Ki);
    Sleep(1.5);

    float leftSpeed, rightSpeed;            //Set up a variable for the speed on each side, since we're going to control them independantly
    leftSpeed = rightSpeed = inputSpeed;    //Start with both sides at the same speed we passed in

    right_encoder.ResetCounts();//reset the counts for both encoders
    left_encoder.ResetCounts();

    //DEBUG
    if(DEBUG){
        if(distance > 0){//If forward, show an acknowledgement on the proteus screen
            LCD.WriteLine("Going forward");
        }else if(distance < 0){//If backward, show an acknowledgement on the proteus screen
            LCD.WriteLine("Going backwards");
        }else{
            LCD.WriteLine("Improper distance parameter");
        }
    }

    //Variable insantiation for PID
    float lastTime = TimeNow();                                                                                         //Previous timestamp. Used to calculate dt
    int leftLastCounts = left_encoder.Counts();                                                                         //Previous encoder positions, used to measure the change in position
    int rightLastCounts = right_encoder.Counts();
    int leftCurrentCounts, rightCurrentCounts;                                                                          //The current position of the encoders. Used to calculate change in position, and thus, rpm
    double lRealSpeed, rRealSpeed, currTime, dt, lError, rError, lInt, rInt, lDeriv, rDeriv, lPrevError, rPrevError;     //A bunch of other variables used in the PID loop
    lInt = 1;
    Sleep(0.001);       //This somehow fixes things


    //Loop until we've measured the desired distance in encoder counts
    while(left_encoder.Counts() < counts){                  // && right_encoder.Counts() < counts
        //Update a bunch of variables to reflect the current state of the robot for PID calculations
        leftCurrentCounts = left_encoder.Counts();
        rightCurrentCounts = right_encoder.Counts();
        currTime = TimeNow();
        dt = currTime-lastTime;

        lRealSpeed = ((leftCurrentCounts-leftLastCounts)/318.0)/(dt/60.0);              //Calculate RPMs for the current iteration

    //PID for the left side
        lError = inputSpeed - lRealSpeed;                       //Proportional part of PID. Calculate the current error value
        lInt = lInt + (lError*dt);                              //Integral portion of PID. Sums error based on time with a reinmann sum
        lDeriv = (lError - lPrevError)/dt;                      //Derivative portion of PID. Change in error/change in time is calculated here
        //Take all these values and set the speed to this
        leftSpeed = ((Kp*lError) + (Ki*lInt)+ (Kd*lDeriv));
        LCD.SetFontColor(CRIMSON);
        LCD.Write(((double)(((lError))*10))/10);
        LCD.Write("\t");
        LCD.SetFontColor(LIME);
        LCD.Write(((double)(lInt*10))/10);
        LCD.SetFontColor(RED);
        LCD.WriteLine((lError*dt));
        LCD.Write("\t");

        //Set motors to calculated values. Have to convert back to percentage from RPMs
        leftmotor.SetPercent((leftSpeed*1/2.25));
        //rightmotor.SetPercent(rightSpeed*100/225);

        if(lInt>100){
            lInt = 100;
        }else if(lInt<-100){
            lInt = -100;
        }

        //Move values from 'current' variables to 'last' variables to prep for next iteration
        lastTime = currTime;
        leftLastCounts = leftCurrentCounts;
        lPrevError = lError;

        //DEBUG
        if(DEBUG){
            //MeasuredSpeed    Error   OutputSpeed

            /*LCD.SetFontColor(GREEN);
            LCD.Write(leftCurrentCounts-leftLastCounts);
            LCD.Write("\t");
            LCD.SetFontColor(BLUE);
            LCD.WriteLine(dt);*/

            //LCD.SetFontColor(GREEN);
            //LCD.WriteLine((int)(leftSpeed*1/2.25));
        }
        Sleep(10);
    }


    rightmotor.Stop();//stop motors
    leftmotor.Stop();
    Sleep(50);//wait for 50 ms to allow for some downtime so nothing weird happens
}


void turn(int direction, int angle){//direction 0 is left, direction 1 is right, angle is from 0 to 360 degrees
    int counts = angle*COUNTS_PER_DEGREE;//counts here is the equivalent of the angle times the defined constant COUNTS_PER_DEGREE

    right_encoder.ResetCounts();//Reset counts for both encoders
    left_encoder.ResetCounts();
    if (direction == 0){//right
        LCD.WriteLine("Turning Right");
        leftmotor.SetPercent(21.);//Set turn percent to negative for the left motor, positive for the right motor to turn in place
        rightmotor.SetPercent(-18.);
        while(left_encoder.Counts() < counts){LCD.WriteLine(left_encoder.Counts());}//continue while the average encoder counts is less than counts
        rightmotor.Stop();//Stop both motors
        leftmotor.Stop();


    }
   else if (direction == 1){//left
        LCD.WriteLine("Turning Left");
        leftmotor.SetPercent(-21.);//Set turn percent to negative for the right motor and positive for the left motor
        rightmotor.SetPercent(18.);
        while(left_encoder.Counts() < counts){LCD.WriteLine(left_encoder.Counts());}//continue while average encoder counts < counts
        rightmotor.Stop();//Stop both motors
        leftmotor.Stop();
    }
    Sleep(50);//Sleep Check
}


void detectColorDDR(){//red is 1, blue is 2. This function is to detect the light at DDR and act accordingly.
    if(photoresis.Value()<1){
        turn(1,90);
        move(0.3,-2);
        Sleep(5.0);
        ddrCheck = 1;
    }else if (photoresis.Value()>1){
       // move(0.3,5);
        turn(1,90);
        move(0.3,-2);
        Sleep(5.0);
    }
}


void instructionSet(){//This function is the instruction set that is a list of instructions

}


int main(void){//The main function is intentionally bare to make things easy to read
    //instructionSet();
    move(50.,318000);
}
