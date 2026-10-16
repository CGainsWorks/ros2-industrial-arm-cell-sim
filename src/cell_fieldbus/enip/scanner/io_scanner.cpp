// A PLC's side of the cell's implicit I/O, for the integration test.
//
// Built on EIPScanner (https://github.com/nimbuscontrols/EIPScanner), an
// open-source EtherNet/IP scanner in C++. It opens one Class 1
// exclusive-owner connection to the cell adapter: output assembly 150 O->T,
// input assembly 100 T->O, configuration assembly 151, 20 ms RPI each way,
// point to point. Then it is driven by tests/test_enip.py over stdin and
// stdout, one line each:
//
//   stdin   out <8 hex digits>   set the 4-byte output assembly
//           close                Forward_Close, then exit
//           quit                 exit without closing: the PLC "crashes",
//                                and the adapter has to time the connection out
//   stdout  @@ open              the connection is up
//           @@ in <64 hex digits> the input assembly, printed when it changes
//           @@ closed            the connection closed or timed out
//
// Protocol lines carry an "@@" prefix so they cannot be confused with the
// library's own log output.
//
//   io_scanner <adapter IPv4 address>

#include <poll.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "ConnectionManager.h"
#include "SessionInfo.h"
#include "cip/connectionManager/NetworkConnectionParams.h"
#include "utils/Logger.h"

using eipScanner::ConnectionManager;
using eipScanner::SessionInfo;
using eipScanner::cip::connectionManager::ConnectionParameters;
using eipScanner::cip::connectionManager::NetworkConnectionParams;
using eipScanner::utils::Logger;
using eipScanner::utils::LogLevel;

namespace {

constexpr int kInputAssembly = 100;
constexpr int kOutputAssembly = 150;
constexpr int kConfigAssembly = 151;
constexpr int kInputSize = 32;
constexpr int kOutputSize = 4;
constexpr int kRpiMicroseconds = 20000;

std::string toHex(const std::vector<uint8_t> &data) {
  std::ostringstream stream;
  for (auto byte : data) {
    stream << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte);
  }
  return stream.str();
}

bool fromHex(const std::string &text, std::vector<uint8_t> &out) {
  if (text.size() != 2 * static_cast<size_t>(kOutputSize)) {
    return false;
  }
  out.clear();
  for (size_t index = 0; index < text.size(); index += 2) {
    out.push_back(static_cast<uint8_t>(std::stoul(text.substr(index, 2), nullptr, 16)));
  }
  return true;
}

void emit(const std::string &line) {
  std::cout << "@@ " << line << std::endl;
}

}  // namespace

int main(int argc, char **argv) {
  if (argc != 2) {
    std::cerr << "usage: io_scanner <adapter IPv4 address>" << std::endl;
    return EXIT_FAILURE;
  }
  Logger::setLogLevel(LogLevel::WARNING);

  auto session = std::make_shared<SessionInfo>(argv[1], 0xAF12);
  ConnectionManager connectionManager;

  ConnectionParameters parameters;
  parameters.connectionPath = {0x20, 0x04, 0x24, kConfigAssembly, 0x2C, kOutputAssembly, 0x2C, kInputAssembly};
  parameters.o2tRealTimeFormat = true;  // 32-bit run/idle header, Run.
  parameters.originatorVendorId = 0xFFFF;
  parameters.originatorSerialNumber = 0x00C0FFEE;
  parameters.t2oNetworkConnectionParams |= NetworkConnectionParams::P2P;
  parameters.t2oNetworkConnectionParams |= NetworkConnectionParams::SCHEDULED_PRIORITY;
  parameters.t2oNetworkConnectionParams |= kInputSize;
  parameters.o2tNetworkConnectionParams |= NetworkConnectionParams::P2P;
  parameters.o2tNetworkConnectionParams |= NetworkConnectionParams::SCHEDULED_PRIORITY;
  parameters.o2tNetworkConnectionParams |= kOutputSize;
  parameters.o2tRPI = kRpiMicroseconds;
  parameters.t2oRPI = kRpiMicroseconds;
  parameters.transportTypeTrigger |= NetworkConnectionParams::CLASS1;

  auto io = connectionManager.forwardOpen(session, parameters);
  auto connection = io.lock();
  if (!connection) {
    emit("failed");
    return EXIT_FAILURE;
  }

  bool closed = false;
  std::string last;
  connection->setDataToSend(std::vector<uint8_t>(kOutputSize, 0));
  connection->setReceiveDataListener([&last](auto, auto, auto data) {
    std::string hex = toHex(data);
    if (hex != last) {
      last = hex;
      emit("in " + hex);
    }
  });
  connection->setCloseListener([&closed]() { closed = true; });
  emit("open");

  std::string pending;
  bool closeRequested = false;
  bool quitRequested = false;
  while (!closed && !closeRequested && !quitRequested) {
    connectionManager.handleConnections(std::chrono::milliseconds(2));

    pollfd input{STDIN_FILENO, POLLIN, 0};
    if (poll(&input, 1, 0) > 0 && (input.revents & (POLLIN | POLLHUP))) {
      char buffer[256];
      ssize_t count = read(STDIN_FILENO, buffer, sizeof(buffer));
      if (count <= 0) {
        quitRequested = true;  // stdin closed: treat the test going away as a crash.
        break;
      }
      pending.append(buffer, static_cast<size_t>(count));
      size_t newline;
      while ((newline = pending.find('\n')) != std::string::npos) {
        std::string line = pending.substr(0, newline);
        pending.erase(0, newline + 1);
        if (line.rfind("out ", 0) == 0) {
          std::vector<uint8_t> data;
          if (fromHex(line.substr(4), data)) {
            connection->setDataToSend(data);
          } else {
            std::cerr << "bad output data: " << line << std::endl;
          }
        } else if (line == "close") {
          closeRequested = true;
        } else if (line == "quit") {
          quitRequested = true;
        }
      }
    }
  }

  if (closeRequested && !closed) {
    connectionManager.forwardClose(session, io);
  }
  emit("closed");
  return EXIT_SUCCESS;
}
