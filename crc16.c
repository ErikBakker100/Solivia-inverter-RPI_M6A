unsigned int calc_crc(unsigned char *sop, unsigned char *eop){ 
	unsigned int crc; 
	unsigned char bit_count; 
	unsigned char *char_ptr;
	 
	char_ptr = sop; 
	 
	crc = 0x0000; //initialize to zero, not all 1's 
	 
	do{ 
		crc^=((*char_ptr)&0x00ff); //make sure only 8-bits get modified 
		bit_count = 0; 
		do{ 
			if(crc&0x0001){ //test before shifting 
				crc>>=1; 
				crc^=0xA001; //reflected version of poly:0x8005 
			}else{ 
				crc>>=1; 
			} 
		} while(bit_count++ < 7); //for every bit 
	} while(char_ptr++ < eop); //for every byte 
	return crc; //return 16 bits of crc 
}
