#include "mkpackage.h"

#include <cstdarg> //va_list
#include <algorithm> //find

#include <iostream> //cout
using namespace std;

namespace mavhub {

// ----------------------------------------------------------------------------
// MKPackage
// ----------------------------------------------------------------------------
MKPackage::MKPackage(uint8_t addr, uint8_t cmd, const uint8_t *data, int length) : addr(addr), cmd(cmd), payload(data, data+length) {

	encode();
}

MKPackage::MKPackage(uint8_t addr, uint8_t cmd, int numofdata, ...) : addr(addr), cmd(cmd) {

	va_list list;
	va_start(list, numofdata);

	uint8_t* data;
	int length;
	for(int i=0; i<numofdata; i++) {
		data = va_arg(list, uint8_t*);
		length = va_arg(list, int);

		payload.insert(payload.end(), data, data+length);
	}
	va_end(list);

	encode();
}

MKPackage::MKPackage(const uint8_t *data, int length) throw(const char*) : encoded(data, data+length) {

	try{
		decode();
	}
	catch(const char *message) {
		throw "ERROR: Input data of MKPackage is not valid";
		return;
	}
	addr = encoded.at(1)-'a';
	cmd = encoded.at(2);
}

MKPackage::~MKPackage() { }

bool MKPackage::checkCRC() const {
	unsigned int i;
	int crc = 0;

	// package #??\r without data is ok
	if( (encoded.at(0) == '#')
	 && (encoded.at(3) == '\r') ) {
		return true;
	}

	// calc crc
	for(i = 0; ( (i < (encoded.size()-2)) && (encoded.at(i+2) != '\r') ); i++) {
		crc += encoded.at(i);
	}
	crc %= 4096;

	// check crc
	if( (encoded.at(i++) == (crc/64 + '='))
	 && (encoded.at(i) == (crc%64 + '=')) ) {
		return true;
	} else {
		return false;
	}
}

void MKPackage::encode() {
	int tmp1, tmp2, tmp3;
	int ptrIn = 0;

	encoded.clear();

	// fill first bytes
	encoded.push_back('#');
	encoded.push_back('a'+addr);
	encoded.push_back(cmd);

	int len = payload.size();

	// pseudo base64 encoding
	while(len>0)  {
		if(len) {
			tmp1 = payload.at(ptrIn++);
			len--;
		} else {
			tmp1 = 0;
		}
		if(len) {
			tmp2 = payload.at(ptrIn++);
			len--;
		} else {
			tmp2 = 0;
		}
		if(len) {
			tmp3 = payload.at(ptrIn++);
			len--;
		} else {
			tmp3 = 0;
		}
		encoded.push_back( '=' + (tmp1 >> 2) );
		encoded.push_back( '=' + ( ((tmp1 & 0x03) << 4) | ((tmp2 & 0xf0) >> 4) ) );
		encoded.push_back( '=' + ( ((tmp2 & 0x0f) << 2) | ((tmp3 & 0xc0) >> 6) ) );
		encoded.push_back( '=' + (tmp3 & 0x3f) );
	}

	// calc. crc
	int crc = 0;
	for(unsigned int i=0; i < encoded.size(); i++) {
		crc += encoded.at(i);
	}
	crc %= 4096;

	// add crc
	encoded.push_back(crc/64 + '=');
	encoded.push_back(crc%64 + '=');

	// add termination symbol
	encoded.push_back('\r');
}

void MKPackage::decode() throw(const char*) {

	if(encoded.size() < 4			//package has at least 4 bytes 
	|| encoded.at(0) != '#'			//package has to start with #
	|| encoded.at(encoded.size()-1) != '\r'	//package has to stop with \r
	|| !checkCRC() ){			//checksum must be ok
		throw "ERROR: MKPackage::decode";
		return;
	}
	int tmp1, tmp2, tmp3, tmp4;
	int ptrIn = 3,
		ptrOut = 0;

	int len = encoded.size() - 3;
	int maxPtr = len - 4;
	while(ptrIn <= maxPtr) {
		tmp1 = encoded.at(ptrIn++) - '=';
		tmp2 = encoded.at(ptrIn++) - '=';
		tmp3 = encoded.at(ptrIn++) - '=';
		tmp4 = encoded.at(ptrIn++) - '=';

		if(len--) {
			payload[ptrOut++] = ( (tmp1 << 2) | (tmp2 >> 4) );
		} else {
			break;
		}
		if(len--) {
			payload[ptrOut++] = ( ((tmp2 & 0x0f) << 4) | (tmp3 >> 2) );
		} else {
			break;
		}
		if(len--) {
			payload[ptrOut++] = ( ((tmp3 & 0x03) << 6) | tmp4 );
		} else {
			break;
		}
	}
}

} //namespace mavhub
