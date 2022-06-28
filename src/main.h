
//-----------STATES-----------
void state0();
void state1();
void state2();

//--------TRANSITIONS---------
bool transitionS0S1();
bool transitionS1S0();
bool transitionS1S2();
bool transitionS2S0();
bool transitionS2S1();

//---------FUNCTIONS----------
int joystickRead();
void move(int sign_, int delta_, unsigned int time_, String contexte);
void stop();