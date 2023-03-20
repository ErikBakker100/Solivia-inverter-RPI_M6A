// *****************************************************************
// *          ESPHome Custom Component Modbus sniffer for          *
// *              Delta Solvia Inverter RPI M6A               *
// *****************************************************************

#include "esphome.h"

/*
There are three possible communication protocols
1) Packet 1: Master to slave 
	Byte # 	Data byte 		Description 
	1 		‘STX’ 			Start of protocol 
	2 		‘ENQ’ 			Master sending request 
	3 		address 		Address of receiving device 
	4 		# of data bytes Number of data included commands 
	5 		command 		Command send to slave 
	6 		sub command 	Sub command send to slave 
				N data N bytes of data 
				N + 1 CRC low byte Low byte of checksum 
				N + 2 CRC high byte High byte of checksum 
				N + 3 ‘ETX’ End of protocol
				
	example 

2) Packet 2: Slave response with acknowledge to master 
	Byte # 	Data byte 		Description 
	1 		‘STX’ 			Start of protocol 
	2 		‘ACK’ 			Slave acknowledging 
	3 		address 		Address of slave responding 
	4 		# of data bytes Number of data included commands 
	5 		command Repeat 	command being responded to 
	6 		sub command Rep sub command being responded to 
				N data N bytes of data 
				N + 1 CRC low byte Low byte of checksum 
				N + 2 CRC high byte High byte of checksum 
				N + 3 ‘ETX’ End of protocol 

3) Packet 3: Slave response with no acknowledge to master 
	Byte # 	Data byte 		Description 
	1 		‘STX’ 			Start of protocol 
	2 		‘NAK’ 			Slave not acknowledging 
	3 		address 		Address of slave responding 
	4 		# of data bytes Number of data included commands 
	5 		command Repeat 	command being responded to 
	6 		sub command Rep sub command being responded to 
	7 		crc low byte 	Low byte of checksum 
	8 		crc high byte 	High byte of checksum 
	9 		‘ETX’ End of protocol 

The checksum is the result from a CRC16 algorithm not including the framing (‘STX’ and ‘ETX’) characters. The CRC16 is calculated by the following polynomial: X16 + X15 + X2 + 1. 
The slave answers with NAK only if the command or subcommand is invalid. 
 */

define ADDRESS 0x01 // address of the slave unit (Solivia Inverter address)

// Protocol Characters
define ENQ 0x05 // Link request (Master only) 
define ACK 0x06 // Slave accepting link (Slave only) 
define NAK 0x15 // Slave not accepting link (Slave only) 
define STX 0x02 // Message start
define ETX 0x03 // Message end

class soliviam6a : public PollingComponent, public Sensor, public UARTDevice {
  public:
    soliviam6a(UARTComponent *parent) : PollingComponent(600), UARTDevice(parent) {}
    Sensor *part = new Sensor();			// SAP part number 11 Byte ASCII 
    Sensor *serial = new Sensor();			// SAP serial number 13 Byte ASCII 
    Sensor *ac_v1 = new Sensor();			// AC Voltage(Phase1) V 2(MSB,LSB) Value/10
    Sensor *ac_a1 = new Sensor();			// AC Current(Phase1) A 2(MSB,LSB) Value/100
    Sensor *ac_p1 = new Sensor();			// AC Power(Phase1) W 2(MSB,LSB)
    Sensor *ac_f1 = new Sensor();			// AC Frequency(Phase1) Hz 2(MSB,LSB) Value/100
    Sensor *ac_v2 = new Sensor();			// AC Voltage(Phase2) V 2(MSB,LSB) Value/10 
    Sensor *ac_a2 = new Sensor();			// AC Current(Phase2) A 2(MSB,LSB) Value/100
	Sensor *ac_p2 = new Sensor();			// AC Power(Phase2) W 2(MSB,LSB) 
    Sensor *ac_f2 = new Sensor();			// AC Frequency(Phase2) Hz 2(MSB,LSB) Value/100 
    Sensor *ac_v3 = new Sensor();			// AC Voltage(Phase3) V 2(MSB,LSB) Value/10
    Sensor *ac_a3 = new Sensor();			// AC Current(Phase3) A 2(MSB,LSB) Value/100
    Sensor *ac_p3 = new Sensor();			// AC Power(Phase3) W 2(MSB,LSB)
    Sensor *ac_f3 = new Sensor();			// AC Frequency(Phase3) Hz 2(MSB,LSB) Value/100
    Sensor *dc_v1 = new Sensor();			// Solar Voltage at Input 1 V 2(MSB,LSB) Value/10 
    Sensor *dc_a1 = new Sensor();			// Solar Current at Input 1 A 2(MSB,LSB) Value/100
    Sensor *dc_p1 = new Sensor();			// Solar Power at Input 1 W 2(MSB,LSB) 
    Sensor *dc_v2 = new Sensor();			// Solar Voltage at Input 2 V 2(MSB,LSB) Value/10 
    Sensor *dc_a2 = new Sensor();			// Solar Current at Input 2 A 2(MSB,LSB) Value/100
    Sensor *dc_p2 = new Sensor();			// Solar Power at Input 2 W 2(MSB,LSB)
    Sensor *ac_p = new Sensor();			// ACPower W 2 (MSB, LSB)
    Sensor *bus_v_pos = new Sensor();		// (+) Bus Voltage V 2(MSB,LSB) Value/10
    Sensor *bus_v_min = new Sensor();		// (-) Bus Voltage V 2(MSB,LSB) Value/10
    Sensor *ac_energy_today = new Sensor();	// Supplied ac energy today Wh 4(MSB,LSB)
    Sensor *runtime_today = new Sensor();	// Inverter runtime today second 4(MSB,LSB) 
    Sensor *ac_energy_total = new Sensor();	// upplied ac energy (total) kWh 4(MSB,LSB) 
    Sensor *runtime_total = new Sensor();	// Inverter runtime(total) second 4(MSB,LSB)
    Sensor *temp_int = new Sensor();		// Calculated temperature inside rack °C 2(MSB,LSB) Signed int16 
    Sensor *status_ac_out1 = new Sensor();	// Status AC Output 1 1 Byte 
    Sensor *status_ac_out2 = new Sensor();	// Status AC Output 2 1 Byte
    Sensor *status_ac_out3 = new Sensor();	// Status AC Output 3 1 Byte
    Sensor *status_ac_out4 = new Sensor();	// Status AC Output 4 1 Byte
    Sensor *status_dc_in1 = new Sensor();	// Status DC Input 1 1 Byte
    Sensor *status_dc_in2 = new Sensor();	// Status DC Input 2 1 Byte
    Sensor *err = new Sensor();				// Error Status 1 Byte
    Sensor *err_ac1 = new Sensor();			// Error Status AC 1 1 Byte
    Sensor *err_g1 = new Sensor();			// Global Error 1 1 Byte 
    Sensor *err_cpu = new Sensor();			// CPU Error 1 Byte
    Sensor *err_g2 = new Sensor();			// Global Error 2 1 Byte
    Sensor *limit_ac_out1 = new Sensor();	// Limits AC output 1 1 Byte
    Sensor *limit_ac_out2 = new Sensor();	// Limits AC output 2 1 Byte
    Sensor *err_g3 = new Sensor();			// Global Error 3 1 Byte
    Sensor *limit_dc1 = new Sensor();		// Limits DC 1 1 Byte
    Sensor *limit_dc2 = new Sensor();		// Limits DC 2 1 Byte 
        
  void setup() override {

  }

  std::vector<int> bytes;

  //void loop() override {
void update() {
    while(available() > 0) {
		bytes.push_back(read());      
		//make sure at least 8 header bytes are available for check
		if(bytes.size() < 8) continue;
		//ESP_LOGD("custom", "Checking for inverter package");
		// Check for Delta Solivia Gateway package response.
		if(bytes[0] != STX && bytes[1] != ACK && bytes[2] != ADDRESS) {
		bytes.erase(bytes.begin()); //remove first byte from buffer
		//buffer will never get above 8 until the response is a match
		continue;
		}
		if (bytes.size() == 158) {
			TwoByte ac_v1_data;
			ac_v1_data.Byte[0] = bytes[0x38];
			ac_v1_data.Byte[1] = bytes[0x39];
			TwoByte ac_a1_data;
			ac_a1_data.Byte[0] = bytes[0x3a];
			ac_a1_data.Byte[1] = bytes[0x3b];
			TwoByte ac_p1_data;
			ac_p1_data.Byte[0] = bytes[0x3c];
			ac_p1_data.Byte[1] = bytes[0x3d];
			TwoByte ac_f1_data;
			ac_f1_data.Byte[0] = bytes[0x3e];
			ac_f1_data.Byte[1] = bytes[0x3f];
			TwoByte ac_v2_data;
			ac_v2_data.Byte[0] = bytes[0x44];
			ac_v2_data.Byte[1] = bytes[0x45];
			TwoByte ac_a2_data;
			ac_a2_data.Byte[0] = bytes[0x46];
			ac_a2_data.Byte[1] = bytes[0x47];
			TwoByte ac_p2_data;
			ac_p2_data.Byte[0] = bytes[0x48];
			ac_p2_data.Byte[1] = bytes[0x49];
			TwoByte ac_f2_data;
			ac_f2_data.Byte[0] = bytes[0x4a];
			ac_f2_data.Byte[1] = bytes[0x4b];
			TwoByte ac_v3_data;
			ac_v3_data.Byte[0] = bytes[0x50];
			ac_v3_data.Byte[1] = bytes[0x51];
			TwoByte ac_a3_data;
			ac_a3_data.Byte[0] = bytes[0x52];
			ac_a3_data.Byte[1] = bytes[0x53];
			TwoByte ac_p3_data;
			ac_p3_data.Byte[0] = bytes[0x54];
			ac_p3_data.Byte[1] = bytes[0x55];
			TwoByte ac_f3_data;
			ac_f3_data.Byte[0] = bytes[0x56];
			ac_f3_data.Byte[1] = bytes[0x57];
			TwoByte dc_v1_data;
			dc_v1_data.Byte[0] = bytes[0x5c];
			dc_v1_data.Byte[1] = bytes[0x5d];
			TwoByte dc_a1_data;
			dc_a1_data.Byte[0] = bytes[0x5e];
			dc_a1_data.Byte[1] = bytes[0x5f];
			TwoByte dc_p1_data;
			dc_p1_data.Byte[0] = bytes[0x60];
			dc_p1_data.Byte[1] = bytes[0x61];
			TwoByte dc_v2_data;
			dc_v2_data.Byte[0] = bytes[0x62];
			dc_v2_data.Byte[1] = bytes[0x63];
			TwoByte dc_a2_data;
			dc_a2_data.Byte[0] = bytes[0x64];
			dc_a2_data.Byte[1] = bytes[0x65];
			TwoByte dc_p2_data;
			dc_p2_data.Byte[0] = bytes[0x66];
			dc_p2_data.Byte[1] = bytes[0x67];

			char etx;
			etx = bytes[158 +3]; // ETX byte (last byte)

			// Quick and dirty check for package integrity is done, in order to avoid irratic sensor value updates 
			// This effectively blocks out any erroneous sensor updates due to rx package corruption
			// Check if ETX = 3. If not (invalid package), ditch whole package, clear buffer and continue
			if (etx != ETX) {
			  ESP_LOGI("custom", "ETX check failure - NO sensor update! ETX: %i", etx);
			  bytes.clear();
			  continue;
			}
			
			ac_v1->publish_state(ac_v1_data.UInt16);
			ac_a1->publish_state(ac_a1_data.UInt16);
			ac_p1->publish_state(ac_p1_data.UInt16);
			ac_f1->publish_state(ac_f1_data.UInt16);
			ac_v2->publish_state(ac_v2_data.UInt16);
			ac_a2->publish_state(ac_a2_data.UInt16);
			ac_p2->publish_state(ac_p2_data.UInt16);
			ac_f2->publish_state(ac_f2_data.UInt16);
			ac_v3->publish_state(ac_v3_data.UInt16);
			ac_a3->publish_state(ac_a3_data.UInt16);
			ac_p3->publish_state(ac_p3_data.UInt16);
			ac_f3->publish_state(ac_f3_data.UInt16);
			dc_v1->publish_state(dc_v1_data.UInt16);
			dc_a1->publish_state(dc_a1_data.UInt16);
			dc_p1->publish_state(dc_p1_data.UInt16);
			ESP_LOGI("custom", "ETX check OK: %i", etx);
			ESP_LOGI("custom", "Inverter %i", bytes[2]);
			ESP_LOGI("custom", "Daily yield: %i Wh", d_yield_data.UInt16);
			ESP_LOGI("custom", "Current production: %i W", ac_power_data.UInt16);
		}
		bytes.clear();
	}
	else { }    
}
		
typedef union
{
	unsigned char Byte[2];
	int16_t Int16;
	uint16_t UInt16;
	unsigned char UChar;
	char Char;
}TwoByte;};
