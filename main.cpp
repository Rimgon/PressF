//Head Coder: Nishant Sathe
//Teammates: Alex King, Michelle Lee, Hrithik Basu
//Team: Press F To Pay Respects (D5)

//These are the declarations of motors, encoders, and constants used throughout the program
#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHServo.h>

AnalogInputPin photoresis(FEHIO::P0_1);
FEHMotor leftmotor(FEHMotor::Motor1,9.0);//the left motor is on port 0 on the proteus
FEHMotor rightmotor(FEHMotor::Motor0,9.0);//the right motor is on port 1 on the proteus
DigitalEncoder right_encoder(FEHIO::P3_7);//right motor encoder is currently set to first port of 0 bank on proteus
DigitalEncoder left_encoder(FEHIO::P0_0);//left motor encoder is current set to second port of 0 bank on proteus

#define FORWARD_PERCENT -25.0//This defines the default speed at which the robot goes forward
#define COUNTS_PER_INCH 33.74//This defines how far per inch the robot will go in specification with the encoder
#define BACKWARDS_PERCENT 18.0//This defines how fast the robot will go while moving backwards
#define COUNTS_PER_DEGREE 1.7//This defines how many counts the encoder should do per degree

int ddrCheck=0;


void startUp(){//This function waits until the proteus screen has been tapped to start the instruction set

        float x,y;
        LCD.Clear(BLACK);
        LCD.SetFontColor(WHITE);
        LCD.WriteLine("Touch the screen to start");
        while(!LCD.Touch(&x,&y)){} //Wait for screen to be pressed
        while(LCD.Touch(&x,&y)){} //Wait for screen to be unpressed

}


void move(int direction, float distance){//direction 1 is forward, direction -1 is backwards, distance in inches, this function tells the robot which direction to move in and how far
    int counts = distance*COUNTS_PER_INCH;//the number of counts is equal to the distance times the defined constant COUNTS_PER_INCH

    right_encoder.ResetCounts();//reset the counts for both encoders
    left_encoder.ResetCounts();

        if(direction == 1){//If forward, show an acknowledgement on the proteus screen
            LCD.WriteLine("Going forward");
        }else if(direction == -1){//If backward, show an acknowledgement on the proteus screen
            LCD.WriteLine("Going backwards");
        }
        leftmotor.SetPercent(direction * -70.);//Set the left motor to it's appropriate speed and direction
        rightmotor.SetPercent(direction * -67.);//Set the right motor it it's appropriate speed and direction
        while( left_encoder.Counts() < counts){}//Continue moving until the average of the two encoder counts is
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
        move(-1,2);
        Sleep(5.0);
        ddrCheck = 1;
    }else if (photoresis.Value()>1){
       // move(1,5);
        turn(1,90);
        move(-1,2);
        Sleep(5.0);
    }
}


void instructionSet(){//This function is the instruction set that is a list of instructions
   // startUp();
   // turn(0,45);
    //move(1,60);//undershoot
   // turn(1,45);
    Sleep(3.0);
    move(1,4);
    turn(1,67);
    move(1,14);
    turn(0,98);
    move(1,51);
    turn(0,93);
    move(1,25);

}

int main(void)//The main function is intentionally bare to make things easy to read
{
    instructionSet();

}


