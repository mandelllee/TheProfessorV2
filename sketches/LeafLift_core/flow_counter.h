// Flow counter
// 
// counts pulses on an iterrupt to track flow
//
// NOTE: currently this is a hack bootstrap impl
//


int flowPin1 = 2;
unsigned long flow1count = 0;

int litersPerMinute = 0;
int litersPerHour = 0;

int flowCounter = 0;

#define TICKS_PER_LITER 450.0


void flowCounterInputHandler()
{
  // total flow
  flow1count += 1;

  flowCounter += 1;

}
int _lastLPM = 0.0;

void setupFlowCounter() {
  pinMode(flowPin1, INPUT_PULLUP);
  attachInterrupt(flowPin1, flowCounterInputHandler, CHANGE);
}


void flowCounterHandler() {
  Serial.println( "tick[" + String(_tickCount) + "] flow[" + String(flowCounter) + "]" );

  // if we have counted 60 seconds
  if ( _tickCount % 15 == 0 ) {

    Serial.println("Calculating the flow rate: flowCounter=" + String(flowCounter  ) );
    _lastLPM = litersPerMinute;
    litersPerMinute = ( flowCounter / TICKS_PER_LITER ) * 4;
    //TODO: we want to do something better here, this is bootstrapping code... 
    litersPerHour = litersPerMinute * 60;
    
    flowCounter = 0;
    Serial.println("after: flowCounter=" + String(flowCounter  ) );
  }

}

