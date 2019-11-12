#ifndef INCLUDED_Common_h
#define INCLUDED_Common_h

#include <stdio.h>
#include <string>
#include <sstream>
#include <utility>      // std::pair
#include <iostream>
#include <iomanip>      // std::setw
#include <locale>			// std::locale, std::toupper
#include <stdexcept>

#include "smcs_cpp/CameraSDK.h"



namespace common
{
    inline bool StartAcquisition(smcs::IDevice device)
    {
        bool status = device->SetIntegerNodeValue("TLParamsLocked", 1);
        status = status && device->CommandNodeExecute("AcquisitionStart");
        return status;
    }

    inline bool StopAcquisition(smcs::IDevice device)
    {
        bool status =  device->CommandNodeExecute("AcquisitionStop");
        status = status && device->SetIntegerNodeValue("TLParamsLocked", 0);
        return status;
    }


    inline std::string IpAddrToString(UINT32 ipAddress)
    {
        std::stringstream stream;
        stream << ((ipAddress >> 24) & 0xFF) << "." << ((ipAddress >> 16) & 0xFF) << "." << ((ipAddress >> 8) & 0xFF) << "." << ((ipAddress) & 0xFF);
        return stream.str();
    }


    inline std::string GetLastFourFromMAC(UINT64 macAddress)
    {
        std::stringstream stream;
        for (auto elem : { 8, 0 }) {
            UINT32 numb = (UINT32)((macAddress >> elem) & 0xFF);
            if (numb < 16) {
                stream << "0";
            }
            stream << std::hex << numb;

            if (elem != 0) {
                stream << "-";
            }
        }

        std::string result(stream.str());

        std::locale loc;
        for (int i = 0; i < result.size(); ++i) {
            result[i] = std::toupper(result[i], loc);
        }

        return result;
    }

    inline std::string GetCurrentDate()
    {
        auto t = std::time(nullptr);
        std::ostringstream oss;

        #ifdef SMCS_OS_WIN
        struct tm timeinfo;
        localtime_s(&timeinfo, &t);
        oss << std::put_time(&timeinfo, "%Y-%m-%d");
        #endif
        #ifdef SMCS_OS_LINUX
        auto timeinfo = *std::localtime(&t);

        oss << (timeinfo.tm_year + 1900);

        if ((timeinfo.tm_mon + 1) < 10) {
            oss << 0 << (timeinfo.tm_mon + 1);
        }
        else {
            oss << (timeinfo.tm_mon + 1);
        }

        if ((timeinfo.tm_mday + 1) < 10) {
            oss << 0 << (timeinfo.tm_mday + 1);
        }
        else {
            oss << (timeinfo.tm_mday + 1);
        }
        #endif

        auto str = oss.str();

        return str;
    }


    //MAC address conversion to string
    inline std::string MacAddrToString(UINT64 macAddress)
    {
        std::stringstream stream;
        // MAC Address has 8 Bytes
        for (auto elem : { 40, 32, 24, 16, 8, 0 }) {
            UINT32 numb = (UINT32)((macAddress >> elem) & 0xFF);
            if (numb < 16) {
                stream << "0";
            }
            stream << std::hex << numb;

            if (elem != 0) {
                stream << ":";
            }
        }

        std::string result(stream.str());

        std::locale loc;
        for (int i = 0; i < result.size(); ++i) {
            result[i] = std::toupper(result[i], loc);
        }

        return result;
    }

    inline void DisplayDevice(const smcs::IDevice device)
    {
        constexpr int wide = 12;
        std::stringstream ss;
        
        ss << std::left << std::setw(wide) << "Vendor" << std::setw(wide) << "Model" << std::setw(wide) << "IP"
            << std::setw(wide) << "Width" << std::setw(wide) << "Height" << std::setw(wide) << "PacketSize" << "\n";

        std::string text;
        device->GetStringNodeValue("DeviceVendorName", text);
        ss << std::setw(wide) << text;

        device->GetStringNodeValue("DeviceModelName", text);
        ss << std::setw(wide) << text;

        ss << std::setw(wide) << common::IpAddrToString(device->GetIpAddress());

        INT64 value = 0;
        device->GetIntegerNodeValue("Width", value);
        ss << std::setw(wide) << value;

        device->GetIntegerNodeValue("Height", value);
        ss << std::setw(wide) << value;

        device->GetIntegerNodeValue("GevSCPSPacketSize", value);
        ss << std::setw(wide) << (value & 0xFFFF);
        

        std::cout << ss.str() << "\n";
    }

    inline std::pair<int, bool> ReadNumber()
    {
        std::string numberS;
        std::getline(std::cin, numberS);
        int number = 0;

        try {
            number = std::stoi(numberS);
        }
        catch (std::invalid_argument) {
            return{ 0 , false };
        }

        return {number, true};
    }

    inline void WaitForInput()
    {
        while (std::cin.get() != '\n');
    }
    
    
    inline std::pair<int, bool> DisplayMenu(const smcs::DevicesList& devices)
    {
        std::stringstream ss;
        ss << std::setw(5) << "Index" << "    " << std::setw(10) << std::left << "Vendor" << std::setw(16) << "Model"
            << std::setw(18) << "IP" << std::setw(18) << "Subnetmask" << std::setw(20) << "MAC" << std::setw(9) << "Available" << "\n";

        int i = 0;
        for (auto device : devices) {
            std::string available = "No";
            if (!device->IsConnected()) {
                device->Connect();
                if (device->IsConnected()) {
                    available = "Yes";
                    device->Disconnect();
                }
            }

            ss << std::setw(5) << std::right << i << "    " << std::setw(10) << std::left << device->GetManufacturerName()
                << std::setw(16) << device->GetModelName() << std::setw(18) << common::IpAddrToString(device->GetIpAddress()) << std::setw(18)
                << common::IpAddrToString(device->GetSubnetMask()) << std::setw(20) << common::MacAddrToString(device->GetMacAddress()) << std::setw(9) << available << "\n";
            i += 1;
        }

        ss << "\n";
        std::cout << ss.str();

        std::cout << "Select index number to connect to camera, make sure camera is available." << std::endl;

        auto number = ReadNumber();
        if (number.second == false || number.first < 0 || number.first >= devices.size()) {
            return { 0, false };
        }

        return {number.first, true};
    }
}


#endif //!INCLUDED_Common_h
