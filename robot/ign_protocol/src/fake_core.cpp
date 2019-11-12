#include <atomic>
#include <chrono>
#include <cmath> // For std::floor.
#include <csignal>
#include <ignition/msgs.hh>
#include <ignition/transport.hh>
#include <iostream>
#include <string>
#include <thread>

#include <skoda_msgs/msgs/commandmsg.pb.h>
#include <skoda_msgs/msgs/errormsg.pb.h>
#include <skoda_msgs/msgs/statusmsg.pb.h>

using skoda_msgs::msgs::CommandMsg;
using skoda_msgs::msgs::StatusMsg;
/// \brief Flag used to break the publisher loop and terminate the program.
static std::atomic_bool g_terminate(false);

//////////////////////////////////////////////////
/// \brief Function callback executed when a SIGINT or SIGTERM signals are
/// captured. This is used to break the infinite loop that publishes messages
/// and exit the program smoothly.
void signal_handler(int _signal) {
  if (_signal == SIGINT || _signal == SIGTERM)
    g_terminate = true;
}

void CommandMsgCallback(const CommandMsg &msg) {
  std::cerr << msg.msg() << std::endl;
}

//////////////////////////////////////////////////
int do_ign_stuff() {

  /* These calls fail by with the same error as if they were not there: free():
   * invalid size: 0x00007ffcb74a6f70 */
  /* header.clear_stamp(); */
  /* msg.clear_header(); */

  return 0;
}
