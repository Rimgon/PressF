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
#include <FEHRPS.h>

//Instantiation of devices
AnalogInputPin photoresis(FEHIO::P1_0);
FEHMotor leftmotor(FEHMotor::Motor0,9.0);//the left motor is on port 0 on the proteus
FEHMotor rightmotor(FEHMotor::Motor1,9.0);//the right motor is on port 1 on the proteus
DigitalEncoder left_encoder(FEHIO::P0_0);//left motor encoder is current set to second port of 0 bank on proteus
DigitalEncoder right_encoder(FEHIO::P0_1);//right motor encoder is currently set to first port of 0 bank on proteus
FEHServo tokenServo(FEHServo::Servo7);
FEHServo foosballArm(FEHServo::Servo0);


//Defining constants for use throughout the code
#define FORWARD_PERCENT -25.0//This defines the default speed at which the robot goes forward
#define COUNTS_PER_INCH 33.74//This defines how far per inch the robot will go in specification with the encoder. There are 318 counts in one revolution.
#define BACKWARDS_PERCENT 18.0//This defines how fast the robot will go while moving backwards
#define COUNTS_PER_DEGREE 2.15//This defines how many counts the encoder should do per degree


//PID constants, tweak for tuning
float Kp = 1.5;
float Ki = 5;
float Kd = 0;

int DEBUG = 0;

int ddrCheck=0;

void checkHeading(int degree){//This method checks the current degree of the robot according to RPS and adjusts accordingly
    float finalTime = TimeNow()+5;//timeout of five seconds
    LCD.SetBackgroundColor(PURPLE);
    LCD.WriteLine("Running checkHeading...");//writes to Proteus to signify checkHeading is working
    LCD.Write("Degree: ");
    LCD.WriteLine(RPS.Heading());//Writes current heading onto screen

    while(TimeNow()<finalTime){//while loop conditional is the timeout of five seconds
        if(abs(RPS.Heading()-degree)>1 && RPS.Heading()>-1){//gives one degree of freedom and also makes sure QR code is seen
            if(RPS.Heading()<degree){//Checks if QR code is below given heading
                leftmotor.SetPercent(10.);//starts motors
                rightmotor.SetPercent(-10.);
                Sleep(100);//pulses motors for 100 ms
                rightmotor.Stop();
                leftmotor.Stop();//stops motors
                Sleep(250);//waits 250 ms to let motors completely stop
            }
            else if(RPS.Heading()>degree){//Same as above, except checks if QR code is above given heading in degrees
                    leftmotor.SetPercent(-10.);
                    rightmotor.SetPercent(10.);
                    Sleep(100);
                    rightmotor.Stop();
                    leftmotor.Stop();
                    Sleep(250);
            }
            LCD.Write("Degree: ");
            LCD.WriteLine(RPS.Heading());
         }else{
             break;//if if isn't satisfied break the while loop
         }
    }

    LCD.WriteLine("Finished Check heading");//visual notification that heading has been checked.
}


void startUp(){//This function waits until the proteus screen has been tapped to start the instruction set
    float x,y;
    LCD.Clear(BLACK);
    LCD.SetFontColor(WHITE);
    LCD.WriteLine("Touch the screen to start");
    while(!LCD.Touch(&x,&y)){} //Wait for screen to be pressed
    while(LCD.Touch(&x,&y)){} //Wait for screen to be unpressed
    LCD.SetBackgroundColor(LIME);
    LCD.SetFontColor(BLACK);
    LCD.WriteLine("Touch registered...\nWaiting on light or 30 seconds");//visual notification that robot is in readied mode

   float currentTime=TimeNow();
   float finalTime=TimeNow()+35;//startup timeout is 35 seconds, if light isn't detected robot will start itself
    //button to button is 5.25, between the lights 13.5
    while(photoresis.Value()>.4){//checks for anything other than no light
        currentTime=TimeNow();
        if(currentTime>finalTime){
            break;//this breaks if the timeout goes off and moves into the movement code
        }
    }
}


void move(float speed, float distance){//direction 1 is forward, direction -1 is backwards, distance in inches, this function tells the robot which direction to move in and how far
    int counts = abs(distance)*COUNTS_PER_INCH;//the number of counts is equal to the distance times the defined constant COUNTS_PER_INCH

    float inputSpeed = (speed/100)*225;      //Convert from percentage speed value to a speed in RPMs


    //DEBUG
    if(DEBUG>=1){//DEBUG helps with debugging, higher number is more detail printed to screen, 0 1 and 2 are available
        LCD.Write("Target speed: ");
        LCD.WriteLine(inputSpeed);
        if(distance > 0){//If forward, show an acknowledgement on the proteus screen
            LCD.WriteLine("Going forward");
        }else if(distance < 0){//If backward, show an acknowledgement on the proteus screen
            LCD.WriteLine("Going backwards");
        }else{
            LCD.WriteLine("Improper distance parameter");
        }
        Sleep(1.5);
    }


    float leftSpeed, rightSpeed;//Set up a variable for the speed on each side, since we're going to control them independantly
    leftSpeed = rightSpeed = inputSpeed;//Start with both sides at the same speed we passed in

    right_encoder.ResetCounts();//reset the counts for both encoders
    left_encoder.ResetCounts();


    //Variable insantiation for PID
    float lastTime = TimeNow();   //Previous timestamp. Used to calculate dt (change in time)
    int leftLastCounts = left_encoder.Counts(); //Previous encoder positions, used to measure the change in position
    int rightLastCounts = right_encoder.Counts();
    int leftCurrentCounts, rightCurrentCounts; //The current position of the encoders. Used to calculate change in position, and thus, rpm
    float lRealSpeed, rRealSpeed, currTime, dt, lError, rError, lInt, rInt, lDeriv, rDeriv, lPrevError, rPrevError;     //A bunch of other variables used in the PID loop
    lInt = rInt = 0;
    Sleep(0.001);//This somehow fixes things. I literally have no explanation, it just does.


    //Loop until we've measured the desired distance in encoder counts
    while(left_encoder.Counts() < counts){// && right_encoder.Counts() < counts
    //Update a bunch of variables to reflect the current state of the robot for PID calculations
        leftCurrentCounts = left_encoder.Counts();
        rightCurrentCounts = right_encoder.Counts();
        currTime = TimeNow();
        dt = currTime-lastTime;

        lRealSpeed = ((leftCurrentCounts-leftLastCounts)/318.0)/(dt/60.0);//Calculate RPMs for the current iteration
        rRealSpeed = ((rightCurrentCounts-rightLastCounts)/318.0)/(dt/60.0);//Calculate RPMs for the current iteration

    //PID for the left side
        lError = inputSpeed - lRealSpeed;//Proportional part of PID. Calculate the current error value
        lInt = lInt + (lError*dt);//Integral portion of PID. Sums error based on time with a reinmann sum
        lDeriv = (lError - lPrevError)/dt;//Derivative portion of PID. Change in error/change in time is calculated here
        leftSpeed = ((Kp*lError) + (Ki*lInt) + (Kd*lDeriv));//Take all these values and set the speed to this

    //PID for the right side
        rError = inputSpeed - rRealSpeed;//Proportional part of PID. Calculate the current error value
        rInt = rInt + (rError*dt);//Integral portion of PID. Sums error based on time with a reinmann sum
        rDeriv = (rError - rPrevError)/dt;//Derivative portion of PID. Change in error/change in time is calculated here
        rightSpeed = ((Kp*rError) + (Ki*rInt) + (Kd*rDeriv));//Take all these values and set the speed to this


        //Set motors to calculated values. Have to convert back to percentage from RPMs
        leftmotor.SetPercent((distance/abs(distance))*(leftSpeed*-1/2.25));
        rightmotor.SetPercent((distance/abs(distance))*(rightSpeed*-1/2.25));


        //This DEBUG gives the most detail to get values with PID
        if(DEBUG>=2){
            //lError    lInt    rError  rInt
            LCD.SetFontColor(CRIMSON);
            LCD.Write(((int)(((lError))*10))/10);
            LCD.Write("\t");
            LCD.SetFontColor(LIME);
            LCD.Write(((int)(lInt*10))/10);
            LCD.Write("\t");
            LCD.SetFontColor(RED);
            LCD.Write(((int)(((rError))*10))/10);
            LCD.Write("\t");
            LCD.SetFontColor(GREEN);
            LCD.Write(((int)(rInt*10))/10);
            LCD.Write("\n");
        }

        //Move values from 'current' variables to 'last' variables to prep for next iteration
        lastTime = currTime;
        leftLastCounts = leftCurrentCounts;
        lPrevError = lError;
        rightLastCounts = rightCurrentCounts;
        rPrevError = rError;

        //Delay between iterations to make sure the motors have time to do something before adjusting again
        Sleep(10);
    }


    rightmotor.Stop();//stop motors
    leftmotor.Stop();
    Sleep(100);//wait for 50 ms to allow for some downtime so nothing weird happens
}


void turn(int direction, int angle){//direction 0 is left, direction 1 is right, angle is from 0 to 359 degrees
    int counts = angle*COUNTS_PER_DEGREE;//counts here is the equivalent of the angle times the defined constant COUNTS_PER_DEGREE

    //Our motors are flipped, so negative values mean positive and vice versa

    right_encoder.ResetCounts();//Reset counts for both encoders
    left_encoder.ResetCounts();
    if (direction == 0){//left direction
        LCD.WriteLine("Turning right");
        leftmotor.SetPercent(18);//Set left motor to go back and right to go forward to turn left in place
        rightmotor.SetPercent(-18);
        while((left_encoder.Counts()+right_encoder.Counts())/2 < counts){}//Continue turning while the average encoder counts are less than the necessary counts
        rightmotor.Stop();//Stop both motors
        leftmotor.Stop();


    }
   else if (direction == 1){//right direction
        LCD.WriteLine("Turning Right");
        leftmotor.SetPercent(-18);//Set left motor to go front and right to go back to turn in place right
        rightmotor.SetPercent(18);
        while(left_encoder.Counts() < counts){}//continue while average encoder counts < counts
        rightmotor.Stop();//Stop both motors
        leftmotor.Stop();
    }
    Sleep(100);//Sleep for 100 ms to stop motors completely
}


void runDDR(){//This function handles how DDR is done by the robot


    if(photoresis.Value()<.3){//If the photoresistor sees a voltage < .3 Volts, start the code for a red light
        LCD.SetBackgroundColor(RED);//Signify that the red light code has started

        //This is all of the instructions sent to the motors and RPS to hit the red button and go up the ramp afterwards
        ////////////////////////////////
        turn(0,46);
        move(30,2);
        turn(0,46);
        move(30,-3);
        leftmotor.SetPercent(25);//backs into button for 5 seconds
        rightmotor.SetPercent(25);
        Sleep(5.5);
        move(30,4);
        turn(1, 94);
        move(30,6.25);
        turn(0,92);
        move(30,-3);
        checkHeading(92);
        turn(0,1);
        move(30, 27.5);
        ///////////////////////////////

    }else{//If red light isn't detected, automatically decide to do blue instructions.
        LCD.SetBackgroundColor(BLUE);//indicator that the light is blue
        turn(0,20);        
        checkHeading(20);       
        move(30, 7);       
        turn(0,73);
        checkHeading(90);      
        move(30,-4);      
        leftmotor.SetPercent(25);//Backs into button for 5 seconds
        rightmotor.SetPercent(25);
        Sleep(5.5);      
        move(30,2);      
        checkHeading(88); 
        move(30, 24.5);//goes up ramp
    }
}


void runComp(){//This is the main competition code, it is the master instruction set for what the proteus does during a round of the competition.
    //Move out of starting position
    move(30,7);
    //Turn away from the token machine
    turn(1,139);
    checkHeading(270);
    //back into the token machine
    move(30,-9);
    //Drop off the token
    tokenServo.SetDegree(0);
    Sleep(1.);

    //Go forwards a bit
    move(30,12);

    //face the back wall
    checkHeading(269);

    //Turn parallel to the front of the DDR machines
    turn(0,90);

    //Move up to the light
    move(30,6.5);

    //run the runDDR function that determines what light is on DDR and instructions to run up the lamp
    runDDR();

    LCD.WriteLine("Going up the ramp");

    LCD.WriteLine("Stabilizing...");
    Sleep(1.0);
    checkHeading(89);
    move(20, 8);
    checkHeading(88);
    move(30,18.5);

    //Align for foosball
    LCD.WriteLine("Aligning for foosball");
    move(30, -.42);
    turn(0, 90);
    move(60, -3);
    Sleep(1.0);

    //drop foosball arm
    foosballArm.SetDegree(70);
    Sleep(2.);

    //Carry foosball counters
    LCD.WriteLine("Moving foosball...");
    move(30, 5);
    turn(0, 5);
    move(30, 5.5);
    //Raise arm
    foosballArm.SetDegree(170);
    Sleep(1.);

    //Line up to flip lever
    LCD.WriteLine("Aligning for lever");
    turn(0,3);
    move(20, 8);

    //flip down the arm
    foosballArm.SetDegree(90);
    Sleep(1.0);

    //turn into lever
    turn(0, 40);

    //arm resets to flip lever
    foosballArm.SetDegree(170);
    turn(0, 13);


    //drop arm
    LCD.WriteLine("Raising arm");

    //line up with the bumped ramp
    Sleep(0.5);
    move(20, 11.5);  
    Sleep(0.5);
    turn(0, 38);
    checkHeading(270);

    //Go down the ramp towards home
    LCD.WriteLine("Going home");
    move(20, 10);
    Sleep(1.0);
    checkHeading(270);
    Sleep(250);
    move(20, 12);

    Sleep(1.);
    checkHeading(270);
    move(30,10);
    turn(0,10);
    checkHeading(270);

    //Run into the button
    move(30,20);

    //If the first button hit doesn't work, run this as a backup to hit button again
    move(30, -10);
    Sleep(1.0);
    Sleep(250);
    checkHeading(150);
    move(30, 30);
    LCD.WriteLine("Hitting the button");

}

int main(void){//The main function is intentionally bare to make things easy to read
    RPS.InitializeTouchMenu();//Initialize RPS first, everything else happens afterwards
    startUp();//Go to the startUp method that acts as a gate to the competition code
    runComp();//Run the competition code
}
