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
FEHMotor leftmotor(FEHMotor::Motor1,9.0);//the left motor is on port 0 on the proteus
FEHMotor rightmotor(FEHMotor::Motor0,9.0);//the right motor is on port 1 on the proteus
DigitalEncoder left_encoder(FEHIO::P0_0);//left motor encoder is current set to second port of 0 bank on proteus
DigitalEncoder right_encoder(FEHIO::P0_1);//right motor encoder is currently set to first port of 0 bank on proteus

//Defining constants for use throughout the code
#define FORWARD_PERCENT -25.0//This defines the default speed at which the robot goes forward
#define COUNTS_PER_INCH 33.74//This defines how far per inch the robot will go in specification with the encoder. There are 318 counts in one revolution.
#define BACKWARDS_PERCENT 18.0//This defines how fast the robot will go while moving backwards
#define COUNTS_PER_DEGREE 1.7//This defines how many counts the encoder should do per degree
#define POWER 0.02//This defines the power with which the 'pid' loop corrects itself. Higher is stronger, but prone to wobble

//PID constants, tweak for tuning
#define Kp 0.1
#define Ki 0
#define Kd 0

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

    int counts = distance;

    float inputSpeed = (speed/100)*225;      //Convert from percentage speed value to a speed in RPMs
    LCD.Write("Target speed: ");
    LCD.WriteLine(inputSpeed);

    float leftSpeed, rightSpeed;
    leftSpeed = rightSpeed = inputSpeed;

    right_encoder.ResetCounts();//reset the counts for both encoders
    left_encoder.ResetCounts();

    if(distance > 0){//If forward, show an acknowledgement on the proteus screen
        LCD.WriteLine("Going forward");
    }else if(distance < 0){//If backward, show an acknowledgement on the proteus screen
        LCD.WriteLine("Going backwards");
    }else{
        LCD.WriteLine("Improper distance parameter");
    }


    int lastTime = TimeNow();
    int leftLastCounts = left_encoder.Counts();
    int rightLastCounts = right_encoder.Counts();
    int leftCurrentCounts, rightCurrentCounts;
    float lRealSpeed, rRealSpeed, currTime, dt, lError, rError, lInt, rInt, lDeriv, rDeriv, lPrevError, rPrevError;

    while(left_encoder.Counts() < counts){      // && right_encoder.Counts() < counts
        leftCurrentCounts = left_encoder.Counts();
        rightCurrentCounts = right_encoder.Counts();
        currTime = TimeNow();
        dt = currTime-lastTime;

        lRealSpeed = ((leftCurrentCounts-leftLastCounts)/318)/(dt/60);      //Calculate how fast we're going in the current iteration
        //LCD.SetFontColor(BLUE);
        //LCD.WriteLine(dt);

        //PID for the left side
        lError = leftSpeed - lRealSpeed;
        lInt = lInt + (lError*dt);
        lDeriv = (lError - lPrevError)/dt;
            //Final calculation for corrections
        leftSpeed = Kp*lError + Ki*lInt + Kd*lDeriv;

        //Set motors to calculated values
        leftmotor.SetPercent(leftSpeed*100/225);
        //rightmotor.SetPercent(rightSpeed*100/225);

        //Move values from 'current' variables to 'last' variables to prep for next iteration
        lastTime = currTime;
        leftLastCounts = leftCurrentCounts;
        rightLastCounts = rightCurrentCounts;
        lPrevError = lError;

        LCD.SetFontColor(CRIMSON);
        LCD.Write(lError);
        LCD.Write("\t");
        LCD.SetFontColor(LIME);
        LCD.WriteLine(leftSpeed);
    }

    //LCD.WriteLine((counts/318.0)/((currTime-lastTime)/60.0));          //rotations/time per rotation in min

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
    move(50.,3180);
}
