/*
 * EtherNet/IP adapter application for the simulated arm cell.
 *
 * Built against OpENer (https://github.com/EIPStackGroup/OpENer) in place of
 * its sample application: docker/enip.Dockerfile copies this file over
 * source/src/ports/POSIX/sample_application/sampleapplication.c and builds the
 * stack unmodified. OpENer provides the encapsulation layer, the connection
 * manager, the identity, TCP/IP and Ethernet link objects, and Class 1 and
 * Class 3 connections. This file provides only what is specific to the cell:
 *
 *   assembly 100  input,  T->O, 32 bytes: the cell status block
 *   assembly 150  output, O->T,  4 bytes: the command word from the PLC
 *   assembly 151  configuration, empty
 *   assembly 152  heartbeat for input-only connections
 *   assembly 153  heartbeat for listen-only connections
 *   class 0x64    vendor-specific cell diagnostics object, instance 1
 *
 * The layout is defined once, in cell_fieldbus/core/enip.py, and
 * tests/test_enip.py checks the numbers below against it.
 *
 * THE CELL IS NOT IN THIS FILE
 *
 * The cell's protocol semantics (edge-triggered commands, latched faults, the
 * counting heartbeat) live in cell_fieldbus/core/registers.py and are shared
 * with the Modbus bridge. This adapter is transport only. Every timer tick it
 * sends the output assembly to the Python side over a loopback UDP socket
 * and copies back whatever status block the Python side last sent. That
 * keeps one implementation of the protocol rather than a C one and a Python
 * one that agree until the day they do not.
 *
 * A LOST CONNECTION MEANS NO COMMANDS
 *
 * The output data is only forwarded while an exclusive-owner connection is
 * open and the scanner is in Run. If the connection times out or is closed,
 * or the PLC goes to Program, the cell is told every command bit is zero:
 * `enable` drops and the cell goes idle. Holding the last outputs on a lost
 * connection is how a cell keeps running after the PLC that was supervising
 * it has gone away.
 */

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include "appcontype.h"
#include "cipcommon.h"
#include "cipidentity.h"
#include "cipqos.h"
#include "ciptcpipinterface.h"
#include "nvdata.h"
#include "opener_api.h"
#include "trace.h"

#define CELL_INPUT_ASSEMBLY 100
#define CELL_OUTPUT_ASSEMBLY 150
#define CELL_CONFIG_ASSEMBLY 151
#define CELL_HEARTBEAT_INPUT_ONLY 152
#define CELL_HEARTBEAT_LISTEN_ONLY 153

#define CELL_INPUT_SIZE 32
#define CELL_OUTPUT_SIZE 4

#define CELL_OBJECT_CLASS 0x64
#define CELL_OBJECT_ATTRIBUTES 11
#define CELL_OBJECT_HIGHEST_ATTRIBUTE 101

/* Status words in the input assembly, by register index. */
#define WORD_STATE 0
#define WORD_FAULT 1
#define WORD_HEARTBEAT 2
#define WORD_CYCLE_COUNT 3
#define WORD_SUCCESS_COUNT 4
#define WORD_LAST_CYCLE_MS 5
#define WORD_LAST_FAULT_STAGE 10
#define WORD_PROTOCOL_VERSION 11
#define WORD_SAFETY_STATUS 12

/* Loopback link to the Python side of the adapter. */
#define DEFAULT_CELL_PORT 47100
#define DEFAULT_ADAPTER_PORT 47101

static EipUint8 g_input_data[CELL_INPUT_SIZE];
static EipUint8 g_output_data[CELL_OUTPUT_SIZE];

/* Vendor-specific object attributes, refreshed from the input block. */
static CipUint g_attr_state;
static CipUint g_attr_fault;
static CipUint g_attr_heartbeat;
static CipUint g_attr_cycle_count;
static CipUint g_attr_success_count;
static CipUint g_attr_last_cycle_ms;
static CipUint g_attr_last_fault_stage;
static CipUint g_attr_protocol_version;
static CipUint g_attr_safety_status;
/* Simulation hooks, settable over explicit messaging. A real cell wires the
 * curtain and the safety reset button to the safety controller as hard
 * inputs; nothing on a standard EtherNet/IP network may reset a safety
 * function. These exist so the integration test can break the simulated
 * curtain from the scanner side. */
static CipUsint g_sim_curtain_blocked;
static CipUsint g_sim_safety_reset;

static int g_link_socket = -1;
static struct sockaddr_in g_cell_address;
static int g_open_connections = 0;
static bool g_run_mode = false;

static int env_port(const char *name, int fallback) {
  const char *value = getenv(name);
  if (value == NULL || *value == '\0') {
    return fallback;
  }
  return atoi(value);
}

static CipUint input_word(int index) {
  return (CipUint)(g_input_data[2 * index] | (g_input_data[2 * index + 1] << 8));
}

static void refresh_cell_object(void) {
  g_attr_state = input_word(WORD_STATE);
  g_attr_fault = input_word(WORD_FAULT);
  g_attr_heartbeat = input_word(WORD_HEARTBEAT);
  g_attr_cycle_count = input_word(WORD_CYCLE_COUNT);
  g_attr_success_count = input_word(WORD_SUCCESS_COUNT);
  g_attr_last_cycle_ms = input_word(WORD_LAST_CYCLE_MS);
  g_attr_last_fault_stage = input_word(WORD_LAST_FAULT_STAGE);
  g_attr_protocol_version = input_word(WORD_PROTOCOL_VERSION);
  g_attr_safety_status = input_word(WORD_SAFETY_STATUS);
}

static void open_link(void) {
  g_link_socket = socket(AF_INET, SOCK_DGRAM, 0);
  if (g_link_socket < 0) {
    perror("cell link socket");
    exit(EXIT_FAILURE);
  }
  int flags = fcntl(g_link_socket, F_GETFL, 0);
  fcntl(g_link_socket, F_SETFL, flags | O_NONBLOCK);

  struct sockaddr_in local;
  memset(&local, 0, sizeof(local));
  local.sin_family = AF_INET;
  local.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  local.sin_port = htons((uint16_t)env_port("CELL_ADAPTER_PORT", DEFAULT_ADAPTER_PORT));
  if (bind(g_link_socket, (struct sockaddr *)&local, sizeof(local)) < 0) {
    perror("cell link bind");
    exit(EXIT_FAILURE);
  }

  memset(&g_cell_address, 0, sizeof(g_cell_address));
  g_cell_address.sin_family = AF_INET;
  g_cell_address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  g_cell_address.sin_port = htons((uint16_t)env_port("CELL_LINK_PORT", DEFAULT_CELL_PORT));
}

static void create_cell_object(void) {
  CipClass *cell_class = CreateCipClass(CELL_OBJECT_CLASS,
                                        0, /* non-default class attributes */
                                        7, /* highest class attribute */
                                        2, /* class services */
                                        CELL_OBJECT_ATTRIBUTES,
                                        CELL_OBJECT_HIGHEST_ATTRIBUTE,
                                        3, /* instance services */
                                        1, /* instances */
                                        "cell diagnostics",
                                        1, /* revision */
                                        NULL);
  if (cell_class == NULL) {
    fprintf(stderr, "could not create the cell diagnostics object\n");
    exit(EXIT_FAILURE);
  }
  CipInstance *instance = GetCipInstance(cell_class, 1);
  InsertAttribute(instance, 1, kCipUint, EncodeCipUint, NULL, &g_attr_state, kGetableSingleAndAll);
  InsertAttribute(instance, 2, kCipUint, EncodeCipUint, NULL, &g_attr_fault, kGetableSingleAndAll);
  InsertAttribute(instance, 3, kCipUint, EncodeCipUint, NULL, &g_attr_last_fault_stage, kGetableSingleAndAll);
  InsertAttribute(instance, 4, kCipUint, EncodeCipUint, NULL, &g_attr_cycle_count, kGetableSingleAndAll);
  InsertAttribute(instance, 5, kCipUint, EncodeCipUint, NULL, &g_attr_success_count, kGetableSingleAndAll);
  InsertAttribute(instance, 6, kCipUint, EncodeCipUint, NULL, &g_attr_last_cycle_ms, kGetableSingleAndAll);
  InsertAttribute(instance, 7, kCipUint, EncodeCipUint, NULL, &g_attr_safety_status, kGetableSingleAndAll);
  InsertAttribute(instance, 8, kCipUint, EncodeCipUint, NULL, &g_attr_protocol_version, kGetableSingleAndAll);
  InsertAttribute(instance, 9, kCipUint, EncodeCipUint, NULL, &g_attr_heartbeat, kGetableSingleAndAll);
  InsertAttribute(instance, 100, kCipUsint, EncodeCipUsint, (CipAttributeDecodeFromMessage)DecodeCipUsint,
                  &g_sim_curtain_blocked, kGetableSingle | kSetable);
  InsertAttribute(instance, 101, kCipUsint, EncodeCipUsint, (CipAttributeDecodeFromMessage)DecodeCipUsint,
                  &g_sim_safety_reset, kGetableSingle | kSetable);

  /* Class-level services live on the metaclass, as in OpENer's own objects. */
  CipClass *meta_class = cell_class->class_instance.cip_class;
  InsertService(meta_class, kGetAttributeSingle, &GetAttributeSingle, "GetAttributeSingle");
  InsertService(meta_class, kGetAttributeAll, &GetAttributeAll, "GetAttributeAll");

  InsertService(cell_class, kGetAttributeSingle, &GetAttributeSingle, "GetAttributeSingle");
  InsertService(cell_class, kGetAttributeAll, &GetAttributeAll, "GetAttributeAll");
  InsertService(cell_class, kSetAttributeSingle, &SetAttributeSingle, "SetAttributeSingle");
}

EipStatus ApplicationInitialization(void) {
  memset(g_input_data, 0, sizeof(g_input_data));
  memset(g_output_data, 0, sizeof(g_output_data));

  CreateAssemblyObject(CELL_INPUT_ASSEMBLY, g_input_data, sizeof(g_input_data));
  CreateAssemblyObject(CELL_OUTPUT_ASSEMBLY, g_output_data, sizeof(g_output_data));
  CreateAssemblyObject(CELL_CONFIG_ASSEMBLY, NULL, 0);
  CreateAssemblyObject(CELL_HEARTBEAT_INPUT_ONLY, NULL, 0);
  CreateAssemblyObject(CELL_HEARTBEAT_LISTEN_ONLY, NULL, 0);

  ConfigureExclusiveOwnerConnectionPoint(0, CELL_OUTPUT_ASSEMBLY, CELL_INPUT_ASSEMBLY, CELL_CONFIG_ASSEMBLY);
  ConfigureInputOnlyConnectionPoint(0, CELL_HEARTBEAT_INPUT_ONLY, CELL_INPUT_ASSEMBLY, CELL_CONFIG_ASSEMBLY);
  ConfigureListenOnlyConnectionPoint(0, CELL_HEARTBEAT_LISTEN_ONLY, CELL_INPUT_ASSEMBLY, CELL_CONFIG_ASSEMBLY);

  create_cell_object();
  open_link();

  InsertGetSetCallback(GetCipClass(kCipQoSClassCode), NvQosSetCallback, kNvDataFunc);
  InsertGetSetCallback(GetCipClass(kCipTcpIpInterfaceClassCode), NvTcpipSetCallback, kNvDataFunc);
  return kEipStatusOk;
}

void HandleApplication(void) {
  /* Called every timer tick (10 ms). Forward the commands, take the status. */
  EipUint8 frame[2 + 1 + 1 + 1 + CELL_OUTPUT_SIZE];
  frame[0] = 'C';
  frame[1] = 'O';
  frame[2] = (EipUint8)((g_open_connections > 0 && g_run_mode) ? 1 : 0);
  frame[3] = g_sim_curtain_blocked;
  frame[4] = g_sim_safety_reset;
  if (frame[2]) {
    memcpy(&frame[5], g_output_data, CELL_OUTPUT_SIZE);
  } else {
    memset(&frame[5], 0, CELL_OUTPUT_SIZE);
  }
  sendto(g_link_socket, frame, sizeof(frame), 0, (struct sockaddr *)&g_cell_address, sizeof(g_cell_address));

  EipUint8 reply[2 + CELL_INPUT_SIZE];
  for (;;) {
    ssize_t received = recv(g_link_socket, reply, sizeof(reply), 0);
    if (received < 0) {
      break; /* EAGAIN: nothing waiting. */
    }
    if (received == (ssize_t)sizeof(reply) && reply[0] == 'C' && reply[1] == 'I') {
      memcpy(g_input_data, &reply[2], CELL_INPUT_SIZE);
    }
  }
  refresh_cell_object();
}

void CheckIoConnectionEvent(unsigned int output_assembly_id, unsigned int input_assembly_id,
                            IoConnectionEvent io_connection_event) {
  (void)input_assembly_id;
  if (output_assembly_id != CELL_OUTPUT_ASSEMBLY) {
    return; /* Input-only and listen-only connections carry no commands. */
  }
  switch (io_connection_event) {
    case kIoConnectionEventOpened:
      g_open_connections++;
      printf("cell: exclusive-owner connection opened\n");
      break;
    case kIoConnectionEventTimedOut:
    case kIoConnectionEventClosed:
      if (g_open_connections > 0) {
        g_open_connections--;
      }
      memset(g_output_data, 0, sizeof(g_output_data));
      printf("cell: exclusive-owner connection %s, commands cleared\n",
             io_connection_event == kIoConnectionEventTimedOut ? "timed out" : "closed");
      break;
    default:
      break;
  }
  fflush(stdout);
}

EipStatus AfterAssemblyDataReceived(CipInstance *instance) {
  switch (instance->instance_number) {
    case CELL_OUTPUT_ASSEMBLY:
      /* Forwarded on the next tick by HandleApplication. */
      return kEipStatusOk;
    case CELL_CONFIG_ASSEMBLY:
      return kEipStatusOk;
    default:
      return kEipStatusOk;
  }
}

EipBool8 BeforeAssemblyDataSend(CipInstance *instance) {
  (void)instance;
  return true;
}

EipStatus ResetDevice(void) {
  CloseAllConnections();
  CipQosUpdateUsedSetQosValues();
  return kEipStatusOk;
}

EipStatus ResetDeviceToInitialConfiguration(void) {
  g_tcpip.encapsulation_inactivity_timeout = 120;
  CipQosResetAttributesToDefaultValues();
  ResetDevice();
  return kEipStatusOk;
}

void *CipCalloc(size_t number_of_elements, size_t size_of_element) {
  return calloc(number_of_elements, size_of_element);
}

void CipFree(void *data) {
  free(data);
}

void RunIdleChanged(EipUint32 run_idle_value) {
  g_run_mode = (run_idle_value & 0x0001) != 0;
  if (g_run_mode) {
    CipIdentitySetExtendedDeviceStatus(kAtLeastOneIoConnectionInRunMode);
  } else {
    CipIdentitySetExtendedDeviceStatus(kAtLeastOneIoConnectionEstablishedAllInIdleMode);
  }
  printf("cell: scanner is in %s\n", g_run_mode ? "Run" : "Idle");
  fflush(stdout);
}
