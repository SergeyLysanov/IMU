/*SELECT SENSOR PORT (S1, S2, S3 or S4), and WHEEL DIAMETER (milimeters).*/
const float your_wheel_diameter = 42;

/*This is the Segway Driver code. Place in same directory as this program*/
#include "segway-driver-lv.h"

void checkBTLinkConnected()
{
	if (nBTCurrentStreamIndex >= 0){
		setBluetoothRawDataMode();

		while (!bBTRawMode)
		{
			// Wait for Bluecore to enter raw data mode
			wait1Msec(1);
		}
	  return;  // An existing Bluetooth connection is present.
	}

	//
	// Not connected. Audible notification and LCD error display
	//
	eraseDisplay();
	nxtDisplayCenteredTextLine(3, "BT not");
	nxtDisplayCenteredTextLine(4, "Connected");

}

task main()
{
	checkBTLinkConnected();

  while(true)
  {
  	balancing();
  }

}
