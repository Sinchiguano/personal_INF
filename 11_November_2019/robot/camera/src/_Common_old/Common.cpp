#include "Common.h"


// IPv4 address conversion to string
std::string Common::IpAddrToString(UINT32 ipAddress)
{
	std::stringstream stream;
	UINT32 temp1, temp2, temp3, temp4;
	
	temp1 = ((ipAddress >> 24) & 0xFF);
	temp2 = ((ipAddress >> 16) & 0xFF);
	temp3 = ((ipAddress >> 8) & 0xFF);
	temp4 = ((ipAddress) & 0xFF);
	
	stream << temp1 << "." << temp2 << "." << temp3 << "." << temp4;
	
	return stream.str();
}


//MAC address conversion to string
std::string Common::MacAddrToString(UINT64 macAddress)
{
	std::string strMac;
	char buf[20] = {0};
	UINT32 temp1, temp2, temp3, temp4, temp5, temp6;
	
	// MAC Address has 8 Bytes
	temp1 = (UINT32)((macAddress >> 40) & 0xFF);
	temp2 = (UINT32)((macAddress >> 32) & 0xFF);
	temp3 = (UINT32)((macAddress >> 24) & 0xFF);
	temp4 = (UINT32)((macAddress >> 16) & 0xFF);
	temp5 = (UINT32)((macAddress >> 8) & 0xFF);
	temp6 = (UINT32)((macAddress) & 0xFF);
	
	sprintf(buf,"%2.2X:%2.2X:%2.2X:%2.2X:%2.2X:%2.2X", temp1, temp2, temp3, temp4, temp5, temp6);
	strMac = std::string(buf);

	return strMac;
}