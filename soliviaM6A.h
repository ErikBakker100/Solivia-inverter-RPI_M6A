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

#define ADDRESS 0x01 // address of the slave unit (Solivia Inverter address)

// Protocol Characters
#define ENQ 0x05 // Link request (Master only) 
#define ACK 0x06 // Slave accepting link (Slave only) 
#define NAK 0x15 // Slave not accepting link (Slave only) 
#define STX 0x02 // Message start
#define ETX 0x03 // Message end

uint32_t timer;

class solivia : public PollingComponent, public Sensor, public UARTDevice {

public:
	solivia(UARTComponent *parent) : PollingComponent(600), UARTDevice(parent) {}

	float get_setup_priority() const override { return esphome::setup_priority::LATE; }

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
		timer = millis();
	}

	std::vector<int> bytes;

	typedef union
	{
		unsigned char Byte[2];
		int16_t Int16;
		uint16_t UInt16;
		unsigned char UChar;
		char Char;
	}TwoByte;

	typedef union
	{
		unsigned char Byte[4];
		uint32_t UInt32;
		unsigned char UChar;
		char Char;
	}FourByte;

	void update() override {
		if (timer+5000 > millis()) {
//			delta->publish("delta-solivia", "waiting for data\r\n");
			timer = millis();
		}
		while(available() > 0) {
			bytes.push_back(read());      
			//make sure at least 8 header bytes are available for check
			if(bytes.size() < 8) continue;
			//ESP_LOGD("custom", "Checking for inverter package");
			// Check for Delta Solivia Gateway package response.
			if(bytes[0] != STX || bytes[1] != ACK || bytes[2] != ADDRESS || bytes[3] != 0xA0 || bytes[4] != 0x60 || bytes[5] != 0x01) {
				bytes.erase(bytes.begin()); //remove first byte from buffer
				/*Alternative code if(bytes[0] != STX && bytes[1] != ACK && bytes[2] != ADDRESS) {
				bytes.erase(bytes.begin()); //remove first byte from buffer */
				//buffer will never get above 8 until the response is a match
				continue;
			}
			char databytes = bytes[3];
			if (bytes.size() == 160) {
				timer = millis();
 				TwoByte ac_v1_data;
				ac_v1_data.Byte[0] = bytes[0x32 +6];
				ac_v1_data.Byte[1] = bytes[0x33 +6];
				TwoByte ac_a1_data;
				ac_a1_data.Byte[0] = bytes[0x34 +6];
				ac_a1_data.Byte[1] = bytes[0x35 +6];
				TwoByte ac_p1_data;
				ac_p1_data.Byte[0] = bytes[0x36 +6];
				ac_p1_data.Byte[1] = bytes[0x37 +6];
				TwoByte ac_f1_data;
				ac_f1_data.Byte[0] = bytes[0x38 +6];
				ac_f1_data.Byte[1] = bytes[0x39 +6];
				TwoByte ac_v2_data;
				ac_v2_data.Byte[0] = bytes[0x3e +6];
				ac_v2_data.Byte[1] = bytes[0x3F +6];
				TwoByte ac_a2_data;
				ac_a2_data.Byte[0] = bytes[0x40 +6];
				ac_a2_data.Byte[1] = bytes[0x41 +6];
				TwoByte ac_p2_data;
				ac_p2_data.Byte[0] = bytes[0x42 +6];
				ac_p2_data.Byte[1] = bytes[0x43 +6];
				TwoByte ac_f2_data;
				ac_f2_data.Byte[0] = bytes[0x44 +6];
				ac_f2_data.Byte[1] = bytes[0x45 +6];
				TwoByte ac_v3_data;
				ac_v3_data.Byte[0] = bytes[0x4a +6];
				ac_v3_data.Byte[1] = bytes[0x4b +6];
				TwoByte ac_a3_data;
				ac_a3_data.Byte[0] = bytes[0x4c +6];
				ac_a3_data.Byte[1] = bytes[0x4d +6];
				TwoByte ac_p3_data;
				ac_p3_data.Byte[0] = bytes[0x4e +6];
				ac_p3_data.Byte[1] = bytes[0x4f +6];
				TwoByte ac_f3_data;
				ac_f3_data.Byte[0] = bytes[0x50 +6];
				ac_f3_data.Byte[1] = bytes[0x51 +6];
				TwoByte dc_v1_data;
				dc_v1_data.Byte[0] = bytes[0x56 +6];
				dc_v1_data.Byte[1] = bytes[0x57 +6];
				TwoByte dc_a1_data;
				dc_a1_data.Byte[0] = bytes[0x58 +6];
				dc_a1_data.Byte[1] = bytes[0x59 +6];
				TwoByte dc_p1_data;
				dc_p1_data.Byte[0] = bytes[0x5a +6];
				dc_p1_data.Byte[1] = bytes[0x5b +6];
				TwoByte dc_v2_data;
				dc_v2_data.Byte[0] = bytes[0x5c +6];
				dc_v2_data.Byte[1] = bytes[0x5d +6];
				TwoByte dc_a2_data;
				dc_a2_data.Byte[0] = bytes[0x5e +6];
				dc_a2_data.Byte[1] = bytes[0x5f +6];
				TwoByte dc_p2_data;
				dc_p2_data.Byte[0] = bytes[0x60 +6];
				dc_p2_data.Byte[1] = bytes[0x61 +6];
				TwoByte ac_p_data;
				ac_p_data.Byte[0] = bytes[0x62 +6];
				ac_p_data.Byte[1] = bytes[0x63 +6];
				FourByte ac_energy_today_data;
				ac_energy_today_data.Byte[0] = bytes[0x68 +6];
				ac_energy_today_data.Byte[1] = bytes[0x69 +6];
				ac_energy_today_data.Byte[2] = bytes[0x6a +6];
				ac_energy_today_data.Byte[3] = bytes[0x6b +6];			
				char etx;
				etx = bytes[databytes+6]; // ETX byte (last byte)

				// Quick and dirty check for package integrity is done, in order to avoid irratic sensor value updates 
				// This effectively blocks out any erroneous sensor updates due to rx package corruption
				// Check if ETX = 3. If not (invalid package), ditch whole package, clear buffer and continue
				if (etx != ETX) {
				  ESP_LOGW("custom", "ETX check failure - NO sensor update! ETX: %i", etx);
				  bytes.clear();
				  continue;
				}
/*
				
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
				dc_v2->publish_state(dc_v1_data.UInt16);
				dc_a2->publish_state(dc_a1_data.UInt16);
				dc_p2->publish_state(dc_p1_data.UInt16);
				ESP_LOGI("custom", "ETX check OK: %i", etx);
				ESP_LOGI("custom", "Daily yield: %i Wh", ac_energy_today_data.UInt32);
				ESP_LOGI("custom", "Current production: %i W", ac_p_data);
				delta->publish("ac_v1", "%i", ac_v1_data.UInt16);
				delta->publish("ac_a1", "%i", ac_a1_data.UInt16);
				delta->publish("ac_p1", "%i", ac_p1_data.UInt16);
				delta->publish("ac_f1", "%i", ac_f1_data.UInt16);
				delta->publish("ac_v2", "%i", ac_v2_data.UInt16);
				delta->publish("ac_a2", "%i", ac_a2_data.UInt16);
				delta->publish("ac_p2", "%i", ac_p2_data.UInt16);
				delta->publish("ac_f2", "%i", ac_f2_data.UInt16);
				delta->publish("dc_v1", "%i", dc_v1_data.UInt16);
				delta->publish("dc_a1", "%i", dc_v1_data.UInt16);
				delta->publish("dc_p1", "%i", dc_v1_data.UInt16);
				delta->publish("dc_v2", "%i", dc_v2_data.UInt16);
				delta->publish("dc_a2", "%i", dc_a2_data.UInt16);
				delta->publish("dc_p2", "%i", dc_p2_data.UInt16);
	*/		}
			bytes.clear();
		}
	}
};


