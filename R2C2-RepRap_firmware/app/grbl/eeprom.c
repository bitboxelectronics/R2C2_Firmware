// stubs; not implemented

unsigned char eeprom_get_char( unsigned int addr )
{
	return 0;
}

void eeprom_put_char( unsigned int addr, unsigned char new_value )
{
}

// Extensions added as part of Grbl 


void memcpy_to_eeprom_with_checksum(unsigned int destination, char *source, unsigned int size) {
}

int memcpy_from_eeprom_with_checksum(char *destination, unsigned int source, unsigned int size) {
  return 0;
}

// end of file
