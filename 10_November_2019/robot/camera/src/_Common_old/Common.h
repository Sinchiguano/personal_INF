#ifndef INCLUDED_Common_h
#define INCLUDED_Common_h

#include <sstream>
#include <stdio.h>
#include <string>

//#include "gige_cpp/GigEVisionSDK.h"
#include "smcs_cpp/CameraSDK.h"

#if defined(GIGE_OS_WIN)
// Windows dependent
#include <conio.h>
#include <windows.h>
#define true TRUE
#define false FALSE
#ifndef _kbhit
#define _kbhit kbhit
#endif
#else
// Linux dependent
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdbool.h>
/*
        static int _kbhit(void) {
                struct termios oldt, newt;
                int ch;
                int oldf;

                tcgetattr(STDIN_FILENO, &oldt);
                newt = oldt;
                newt.c_lflag &= ~(ICANON | ECHO);
                tcsetattr(STDIN_FILENO, TCSANOW, &newt);
                oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
                fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

                ch = getchar();

                tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
                fcntl(STDIN_FILENO, F_SETFL, oldf);

                if(ch != EOF) {
                        ungetc(ch, stdin);
                        return 1;
                }

                return 0;
        }
*/
#endif

class Common {
public:
  static std::string IpAddrToString(UINT32 ipAddress);
  static std::string MacAddrToString(UINT64 macAddress);
};

#endif //! INCLUDED_Common_h
