// Zie tevens https://www.sydspost.nl/index.php/2021/03/23/opbrengst-zonnepanelen-uitlezen-2/ voor goede rest uitleg
// https://github.com/lvzon/soliviamonitor/commit/2a62329a322ef6991d6b0c0884d6b90c65df0ebe voor een python script
// https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/

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

#include <CppLinuxSerial/SerialPort.hpp>
#include <mosquitto.h>
#include <sys/time.h>
#include <cstring>

#define ADDRESS 0x01 // address of the slave unit (Solivia Inverter address)

// Protocol Characters
#define ENQ 0x05 // Link request (Master only)
#define ACK 0x06 // Slave accepting link (Slave only)
#define NAK 0x15 // Slave not accepting link (Slave only)
#define STX 0x02 // Message start
#define ETX 0x03 // Message end

#define SERIAL_PORT "/dev/ttyS0"
#define SERIAL_BAUDRATE BaudRate::B_19200 // B_9600, B_19200, B_38400, B_57600, B_115200, B_230400, B_460800

#define MQTT_HOST "localhost"
#define MQTT_PORT 1883
#define MQTT_TOPIC "delta"
#define MQTT_USERNAME "mqtt"
#define MQTT_PASSWORD "1Edereen"

using namespace mn::CppLinuxSerial;

std::string uint8_vector_to_hex_string(const std::vector<uint8_t>&);
uint16_t calc_crc(const std::vector<uint8_t>&);
uint32_t timer;
uint64_t seconds();

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
}FourByte;

int main() {
	// Create serial port object and open serial port at 57600 buad, 8 data bits, no parity bit, one stop bit (8n1),
	// and no flow control
	std::cout << "Opening Serial port" << std::endl;
	SerialPort serialPort(SERIAL_PORT, SERIAL_BAUDRATE, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE);
	serialPort.SetTimeout(100); // Block for up to ms to receive data
	serialPort.Open();
	// Set up MQTT client and connect to server
	std::cout << "Starting MQTT client" << std::endl;
	mosquitto_lib_init();
    mosquitto *mosq = mosquitto_new("delta", true, NULL);
    if (!mosq) {
        std::cerr << "Failed to create Mosquitto object" << std::endl;
        return -1;
    }

	if (mosquitto_username_pw_set(mosq, MQTT_USERNAME, MQTT_PASSWORD) != MOSQ_ERR_SUCCESS) {
		std::cerr << "Failed to set MQTT username and password" << std::endl;
		mosquitto_destroy(mosq);
		mosquitto_lib_cleanup();
		return -1;
	}

    if (mosquitto_connect(mosq, MQTT_HOST, MQTT_PORT, 60) != MOSQ_ERR_SUCCESS) {
        std::cerr << "Failed to connect to MQTT broker" << std::endl;
        mosquitto_destroy(mosq);
        mosquitto_lib_cleanup();
        return -1;
    }

	// Allocate read buffer
	std::vector<uint8_t> bytes;
	// keep track of elapsed seconds
	timer = seconds();
	std::cout << "Listening" << std::endl;

	while (true) {
		if (seconds() > timer+5) {
			const char* topic = MQTT_TOPIC;
			const char* payload =  "delta-solivia : waiting for data\r\n";
			mosquitto_publish(mosq, NULL, topic, strlen(payload), payload, 0, false);
			timer = seconds();
		}
		while (serialPort.Available() > 0) {
			serialPort.ReadBinary(bytes);     
			//make sure at least 8 header bytes are available for check
			if(bytes.size() < 8) continue;
			// Check for Delta Solivia Gateway package response.
			if(bytes[0] != STX || bytes[1] != ACK || bytes[2] != ADDRESS || bytes[4] != 0x60 || bytes[5] != 0x01) {
				bytes.erase(bytes.begin()); //remove first byte from buffer
				//buffer will never get above 8 until the response is a match
				continue;
			}
			if (bytes.size() == bytes[3]+7) { // did we receive the right amount of bytes in the payload + command bytes
 				TwoByte crc;
				crc.Byte[0] = bytes[164];
				crc.Byte[1] = bytes[165];
				if (calc_crc(bytes) == crc.UInt16) { // Is our data not corrupted
					std::cout << "Data = \"" << uint8_vector_to_hex_string(bytes) << "\"" << std::endl;
					char partnr[11];
					for (int x=0; x<11; x++) {
						partnr[x] = bytes[x+6];
						partnr[11]=0;
					}
					std::cout << "part nr = " << partnr << std::endl;
					char serialnr[13];
					for (int x=0; x<13; x++) {
						serialnr[x] = bytes[x+17];
						serialnr[13]=0;
					}
					std::cout << "serial = " << serialnr << std::endl;
					TwoByte ac_v1_data;
					ac_v1_data.Byte[0] = bytes[56];
					ac_v1_data.Byte[1] = bytes[57];
					TwoByte ac_a1_data;
					ac_a1_data.Byte[0] = bytes[58];
					ac_a1_data.Byte[1] = bytes[59];
					TwoByte ac_p1_data;
					ac_p1_data.Byte[0] = bytes[60];
					ac_p1_data.Byte[1] = bytes[61];
					TwoByte ac_f1_data;
					ac_f1_data.Byte[0] = bytes[61];
					ac_f1_data.Byte[1] = bytes[62];
					TwoByte ac_v2_data;
					ac_v2_data.Byte[0] = bytes[66];
					ac_v2_data.Byte[1] = bytes[67];
					TwoByte ac_a2_data;
					ac_a2_data.Byte[0] = bytes[68];
					ac_a2_data.Byte[1] = bytes[69];
					TwoByte ac_p2_data;
					ac_p2_data.Byte[0] = bytes[70];
					ac_p2_data.Byte[1] = bytes[71];
					TwoByte ac_f2_data;
					ac_f2_data.Byte[0] = bytes[72];
					ac_f2_data.Byte[1] = bytes[73];
					TwoByte ac_v3_data;
					ac_v3_data.Byte[0] = bytes[77];
					ac_v3_data.Byte[1] = bytes[78];
					TwoByte ac_a3_data;
					ac_a3_data.Byte[0] = bytes[79];
					ac_a3_data.Byte[1] = bytes[80];
					TwoByte ac_p3_data;
					ac_p3_data.Byte[0] = bytes[81];
					ac_p3_data.Byte[1] = bytes[82];
					TwoByte ac_f3_data;
					ac_f3_data.Byte[0] = bytes[83];
					ac_f3_data.Byte[1] = bytes[84];
					TwoByte dc_v1_data;
					dc_v1_data.Byte[0] = bytes[88];
					dc_v1_data.Byte[1] = bytes[89];
					TwoByte dc_a1_data;
					dc_a1_data.Byte[0] = bytes[90];
					dc_a1_data.Byte[1] = bytes[91];
					TwoByte dc_p1_data;
					dc_p1_data.Byte[0] = bytes[92];
					dc_p1_data.Byte[1] = bytes[93];
					TwoByte dc_v2_data;
					dc_v2_data.Byte[0] = bytes[94];
					dc_v2_data.Byte[1] = bytes[95];
					TwoByte dc_a2_data;
					dc_a2_data.Byte[0] = bytes[96];
					dc_a2_data.Byte[1] = bytes[97];
					TwoByte dc_p2_data;
					dc_p2_data.Byte[0] = bytes[98];
					dc_p2_data.Byte[1] = bytes[99];
					TwoByte ac_p_data;
					ac_p_data.Byte[0] = bytes[100];
					ac_p_data.Byte[1] = bytes[101];
					TwoByte bus_plus_data;
					bus_plus_data.Byte[0] = bytes[102];
					bus_plus_data.Byte[1] = bytes[103];
					TwoByte bus_minus_data;
					bus_minus_data.Byte[0] = bytes[104];
					bus_minus_data.Byte[1] = bytes[105];
					FourByte ac_energy_today_data;
					ac_energy_today_data.Byte[0] = bytes[106];
					ac_energy_today_data.Byte[1] = bytes[107];
					ac_energy_today_data.Byte[2] = bytes[108];
					ac_energy_today_data.Byte[3] = bytes[109];			
					FourByte runtimme_today_data;
					runtimme_today_data.Byte[0] = bytes[110];
					runtimme_today_data.Byte[1] = bytes[111];
					runtimme_today_data.Byte[2] = bytes[112];
					runtimme_today_data.Byte[3] = bytes[113];
					std::cout << "ac_v1 = " << ac_v1_data.UInt16 << std::endl;
					std::cout << "energy today = " << ac_energy_today_data.UInt32 << std::endl;
					std::cout << "runtime today = " << runtimme_today_data.UInt32 << std::endl;				
				} else {
					const char* topic = MQTT_TOPIC;
					const char* payload =  "delta-solivia : wrong data received.\r\n";
					mosquitto_publish(mosq, NULL, topic, strlen(payload), payload, 0, false);
				}
				timer = seconds();
				bytes.clear();
			}
		}
	}

	// Close the serial port
	serialPort.Close();
}

std::string uint8_vector_to_hex_string(const std::vector<uint8_t>& v) {
    std::string result;
    result.resize(v.size() * 2);
    const char letters[] = "0123456789ABCDEF";
    char* current_hex_char = &result[0];
    for (uint8_t b : v) {
        *current_hex_char++ = letters[b >> 4];
        *current_hex_char++ = letters[b & 0xf];
    }
    return result;
}

uint64_t seconds()
{
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return  (uint64_t)tv.tv_sec;
}

uint16_t calc_crc(const std::vector<uint8_t>& v) {
	uint16_t  crc; 
	uint8_t bit_count; 
	uint8_t length = 1;
	crc = 0x0000; //initialize to zero, not all 1's 
	do {
		crc^=((v[length])&0x00ff); //make sure only 8-bits get modified
		bit_count = 0; 
		do {
			if(crc&0x0001) { //test before shifting 
				crc>>=1; 
				crc^=0xA001; //reflected version of poly:0x8005 
			} 
			else {
				crc>>=1; 
			} 
		} while(bit_count++ < 7); //for every bit 
		length++;
	} while(length < v.size()-3); //for every byte 
	return crc; //return 16 bits of crc
} 


/*
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

			char databytes = bytes[3];
			if (bytes.size() == 160) {
				timer = millis();

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
	*/
