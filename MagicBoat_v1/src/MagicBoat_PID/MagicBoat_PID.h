#ifndef MagicBoat_PID_h
#define MagicBoat_PID_h

class MagicBoat_PID
{

  public:

  //Constants used in some of the functions below
  #define AUTOMATIC	1
  #define MANUAL	0
  #define DIRECT  0
  #define REVERSE  1
  #define P_ON_M 0
  #define P_ON_E 1

  //commonly used functions **************************************************************************
    MagicBoat_PID(float*, float*, float*,        // * constructor.  links the MagicBoat_PID to the Input, Output, and 
        float, float, float, int, int, float*,int*);//   Setpoint.  Initial tuning parameters are also set here.
                                          //   (overload for specifying proportional mode)

    MagicBoat_PID(float*, float*, float*,        // * constructor.  links the MagicBoat_PID to the Input, Output, and 
        float, float, float, int, float*,int*);     //   Setpoint.  Initial tuning parameters are also set here
	
    void SetMode(int Mode);               // * sets MagicBoat_PID to either Manual (0) or Auto (non-0)

    bool Compute();                       // * performs the MagicBoat_PID calculation.  it should be
                                          //   called every time loop() cycles. ON/OFF and
                                          //   calculation frequency can be set using SetMode
                                          //   SetSampleTime respectively
										  
	bool ComputeMB();                       // * Edit by MagicBoat Team
                                          //   
                                          //   
                                          //   

    void SetOutputLimits(float, float); // * clamps the output to a specific range. 0-255 by default, but
										                      //   it's likely the user will want to change this depending on
										                      //   the application
	


  //available but not commonly used functions ********************************************************
    void SetTunings(float, float,       // * While most users will set the tunings once in the 
                    float);         	    //   constructor, this function gives the user the option
                                          //   of changing tunings during runtime for Adaptive control
    void SetTunings(float, float,       // * overload for specifying proportional mode
                    float, int);         	  

	void SetControllerDirection(int);	  // * Sets the Direction, or "Action" of the controller. DIRECT
										  //   means the output will increase when error is positive. REVERSE
										  //   means the opposite.  it's very unlikely that this will be needed
										  //   once it is set in the constructor.
    void SetSampleTime(int);              // * sets the frequency, in Milliseconds, with which 
                                          //   the MagicBoat_PID calculation is performed.  default is 100
										  
										  
										  
  //Display functions ****************************************************************
	float GetKp();						  // These functions query the MagicBoat_PID for interal values.
	float GetKi();						  //  they were created mainly for the MagicBoat_PID front-end,
	float GetKd();						  // where it's important to know what is actually 
	int GetMode();						  //  inside the MagicBoat_PID.
	int GetDirection();					  //

  private:
	void Initialize();
	
	float dispKp;				// * we'll hold on to the tuning parameters in user-entered 
	float dispKi;				//   format for display purposes
	float dispKd;				//
    
	float kp;                  // * (P)roportional Tuning Parameter
    float ki;                  // * (I)ntegral Tuning Parameter
    float kd;                  // * (D)erivative Tuning Parameter

	int controllerDirection;
	int pOn;

    float *myInput;              // * Pointers to the Input, Output, and Setpoint variables
    float *myOutput;             //   This creates a hard link between the variables and the 
    float *mySetpoint;           //   MagicBoat_PID, freeing the user from having to constantly tell us
                                  //   what these values are.  with pointers we'll just know.
			  
	unsigned long lastTime;
	float outputSum, lastInput;

	unsigned long SampleTime;
	float outMin, outMax;
	bool inAuto, pOnE;
};
#endif

