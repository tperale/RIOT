/*
 * Copyright (C) 2016 Unwired Devices <info@unwds.com>
 *               2017 Inria Chile
 *               2017 Inria
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     tests
 * @{
 * @file
 * @brief       Test application for SX128X modem driver
 *
 * @author      Eugene P. <ep@unwds.com>
 * @author      José Ignacio Alamos <jose.alamos@inria.cl>
 * @author      Alexandre Abadie <alexandre.abadie@inria.fr>
 * @}
 */

#include <errno.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sx128x_registers.h>

#include "shell.h"
#include "shell_commands.h"
#include "sx128x.h"
#include "thread.h"

#include "net/lora24.h"
#include "net/netdev.h"
#include "net/netdev/lora.h"

#include "board.h"

#include "sx128x_internal.h"
#include "sx128x_netdev.h"
#include "sx128x_params.h"

#include "fmt.h"

#define SX128X_LORA_MSG_QUEUE (16U)
#define SX128X_STACKSIZE (THREAD_STACKSIZE_DEFAULT)

#define MSG_TYPE_ISR (0x3456)

static char stack[SX128X_STACKSIZE];
static kernel_pid_t _recv_pid;

static char message[32];
static sx128x_t sx128x;

int lora_setup_cmd(int argc, char **argv) {
  if (argc < 4) {
    puts("usage: setup "
         "<bandwidth (200, 400, 800, 1600)> "
         "<spreading factor (5..12)> "
         "<code rate (5..8)>");
    return -1;
  }

  /* Check bandwidth value */
  int bw = atoi(argv[1]);
  uint8_t lora_bw;
  switch (bw) {
  case 200:
    puts("setup: setting 200Hz bandwidth");
    lora_bw = LORA24_BW_200_KHZ;
    break;

  case 400:
    puts("setup: setting 400KHz bandwidth");
    lora_bw = LORA24_BW_400_KHZ;
    break;

  case 800:
    puts("setup: setting 800KHz bandwidth");
    lora_bw = LORA24_BW_800_KHZ;
    break;
  case 1600:
    puts("setup: setting 1600KHz bandwidth");
    lora_bw = LORA24_BW_1600_KHZ;
    break;
  default:
    puts("[Error] setup: invalid bandwidth value given, "
         "only 125, 250 or 500 allowed.");
    return -1;
  }

  /* Check spreading factor value */
  uint8_t lora_sf = atoi(argv[2]);
  if (lora_sf < 5 || lora_sf > 12) {
    puts("[Error] setup: invalid spreading factor value given");
    return -1;
  }

  /* Check coding rate value */
  int cr = atoi(argv[3]);
  if (cr < 5 || cr > 8) {
    puts("[Error ]setup: invalid coding rate value given");
    return -1;
  }
  uint8_t lora_cr = (uint8_t)(cr - 4);

  /* Configure radio device */
  netdev_t *netdev = (netdev_t *)&sx128x;
  netdev->driver->set(netdev, NETOPT_BANDWIDTH, &lora_bw, sizeof(lora_bw));
  netdev->driver->set(netdev, NETOPT_SPREADING_FACTOR, &lora_sf,
                      sizeof(lora_sf));
  netdev->driver->set(netdev, NETOPT_CODING_RATE, &lora_cr, sizeof(lora_cr));

  puts("[Info] setup: configuration set with success");

  return 0;
}

int random_cmd(int argc, char **argv) {
  (void)argc;
  (void)argv;

  netdev_t *netdev = (netdev_t *)&sx128x;
  uint32_t rand;
  netdev->driver->get(netdev, NETOPT_RANDOM, &rand, sizeof(rand));
  printf("random: number from sx128x: %u\n", (unsigned int)rand);

  /* reinit the transceiver to default values */
  sx128x_init_radio_settings((sx128x_t *)netdev);

  return 0;
}

int register_cmd(int argc, char **argv) {
  if (argc < 2) {
    puts("usage: register <get | set>");
    return -1;
  }

  if (strstr(argv[1], "get") != NULL) {
    if (argc < 3) {
      puts("usage: register get <all | allinline | regnum>");
      return -1;
    }

    if (strcmp(argv[2], "all") == 0) {
      puts("- listing all registers -");
      uint8_t reg = 0, data = 0;
      /* Listing registers map */
      puts("Reg   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F");
      for (unsigned i = 0; i <= 7; i++) {
        printf("0x%02X ", i << 4);

        for (unsigned j = 0; j <= 15; j++, reg++) {
          data = sx128x_reg_read(&sx128x, reg);
          printf("%02X ", data);
        }
        puts("");
      }
      puts("-done-");
      return 0;
    } else if (strcmp(argv[2], "allinline") == 0) {
      puts("- listing all registers in one line -");
      /* Listing registers map */
      for (uint16_t reg = 0; reg < 256; reg++) {
        printf("%02X ", sx128x_reg_read(&sx128x, (uint8_t)reg));
      }
      puts("- done -");
      return 0;
    } else {
      long int num = 0;
      /* Register number in hex */
      if (strstr(argv[2], "0x") != NULL) {
        num = strtol(argv[2], NULL, 16);
      } else {
        num = atoi(argv[2]);
      }

      if (num >= 0 && num <= 255) {
        printf("[regs] 0x%02X = 0x%02X\n", (uint8_t)num,
               sx128x_reg_read(&sx128x, (uint8_t)num));
      } else {
        puts("regs: invalid register number specified");
        return -1;
      }
    }
  } else if (strstr(argv[1], "set") != NULL) {
    if (argc < 4) {
      puts("usage: register set <regnum> <value>");
      return -1;
    }

    long num, val;

    /* Register number in hex */
    if (strstr(argv[2], "0x") != NULL) {
      num = strtol(argv[2], NULL, 16);
    } else {
      num = atoi(argv[2]);
    }

    /* Register value in hex */
    if (strstr(argv[3], "0x") != NULL) {
      val = strtol(argv[3], NULL, 16);
    } else {
      val = atoi(argv[3]);
    }

    sx128x_reg_write(&sx128x, (uint8_t)num, (uint8_t)val);
  } else {
    puts("usage: register get <all | allinline | regnum>");
    return -1;
  }

  return 0;
}

int send_cmd(int argc, char **argv) {
  if (argc <= 1) {
    puts("usage: send <payload>");
    return -1;
  }

  printf("sending \"%s\" payload (%u bytes)\n", argv[1],
         (unsigned)strlen(argv[1]) + 1);

  iolist_t iolist = {.iol_base = argv[1], .iol_len = (strlen(argv[1]) + 1)};

  netdev_t *netdev = (netdev_t *)&sx128x;
  if (netdev->driver->send(netdev, &iolist) == -ENOTSUP) {
    puts("Cannot send: radio is still transmitting");
  }

  return 0;
}

int listen_cmd(int argc, char **argv) {
  (void)argc;
  (void)argv;

  netdev_t *netdev = (netdev_t *)&sx128x;
  /* Switch to continuous listen mode */
  const netopt_enable_t single = false;
  netdev->driver->set(netdev, NETOPT_SINGLE_RECEIVE, &single, sizeof(single));
  const uint32_t timeout = 0;
  netdev->driver->set(netdev, NETOPT_RX_TIMEOUT, &timeout, sizeof(timeout));

  /* Switch to RX state */
  netopt_state_t state = NETOPT_STATE_RX;
  netdev->driver->set(netdev, NETOPT_STATE, &state, sizeof(state));

  printf("Listen mode set\n");

  return 0;
}

int syncword_cmd(int argc, char **argv) {
  if (argc < 2) {
    puts("usage: syncword <get|set>");
    return -1;
  }

  netdev_t *netdev = (netdev_t *)&sx128x;
  uint8_t syncword;
  if (strstr(argv[1], "get") != NULL) {
    netdev->driver->get(netdev, NETOPT_SYNCWORD, &syncword, sizeof(syncword));
    printf("Syncword: 0x%02x\n", syncword);
    return 0;
  }

  if (strstr(argv[1], "set") != NULL) {
    if (argc < 3) {
      puts("usage: syncword set <syncword>");
      return -1;
    }
    syncword = fmt_hex_byte(argv[2]);
    netdev->driver->set(netdev, NETOPT_SYNCWORD, &syncword, sizeof(syncword));
    printf("Syncword set to %02x\n", syncword);
  } else {
    puts("usage: syncword <get|set>");
    return -1;
  }

  return 0;
}
int channel_cmd(int argc, char **argv) {
  if (argc < 2) {
    puts("usage: channel <get|set>");
    return -1;
  }

  netdev_t *netdev = (netdev_t *)&sx128x;
  uint32_t chan;
  if (strstr(argv[1], "get") != NULL) {
    netdev->driver->get(netdev, NETOPT_CHANNEL_FREQUENCY, &chan, sizeof(chan));
    printf("Channel: %i\n", (int)chan);
    return 0;
  }

  if (strstr(argv[1], "set") != NULL) {
    if (argc < 3) {
      puts("usage: channel set <channel>");
      return -1;
    }
    chan = atoi(argv[2]);
    netdev->driver->set(netdev, NETOPT_CHANNEL_FREQUENCY, &chan, sizeof(chan));
    printf("New channel set\n");
  } else {
    puts("usage: channel <get|set>");
    return -1;
  }

  return 0;
}

int rx_timeout_cmd(int argc, char **argv) {
  if (argc < 2) {
    puts("usage: channel <get|set>");
    return -1;
  }

  netdev_t *netdev = (netdev_t *)&sx128x;
  uint16_t rx_timeout;
  if (strstr(argv[1], "set") != NULL) {
    if (argc < 3) {
      puts("usage: rx_timeout set <rx_timeout>");
      return -1;
    }
    rx_timeout = atoi(argv[2]);
    netdev->driver->set(netdev, NETOPT_RX_SYMBOL_TIMEOUT, &rx_timeout,
                        sizeof(rx_timeout));
    printf("rx_timeout set to %i\n", rx_timeout);
  } else {
    puts("usage: rx_timeout set");
    return -1;
  }

  return 0;
}

int reset_cmd(int argc, char **argv) {
  (void)argc;
  (void)argv;
  netdev_t *netdev = (netdev_t *)&sx128x;
  puts("resetting sx128x...");
  netopt_state_t state = NETOPT_STATE_RESET;
  netdev->driver->set(netdev, NETOPT_STATE, &state, sizeof(netopt_state_t));
  return 0;
}

static void _set_opt(netdev_t *netdev, netopt_t opt, bool val, char *str_help) {
  netopt_enable_t en = val ? NETOPT_ENABLE : NETOPT_DISABLE;
  netdev->driver->set(netdev, opt, &en, sizeof(en));
  printf("Successfully ");
  if (val) {
    printf("enabled ");
  } else {
    printf("disabled ");
  }
  printf("%s\n", str_help);
}

int crc_cmd(int argc, char **argv) {
  netdev_t *netdev = (netdev_t *)&sx128x;
  if (argc < 3 || strcmp(argv[1], "set") != 0) {
    printf("usage: %s set <1|0>\n", argv[0]);
    return 1;
  }

  int tmp = atoi(argv[2]);
  _set_opt(netdev, NETOPT_INTEGRITY_CHECK, tmp, "CRC check");
  return 0;
}

int implicit_cmd(int argc, char **argv) {
  netdev_t *netdev = (netdev_t *)&sx128x;
  if (argc < 3 || strcmp(argv[1], "set") != 0) {
    printf("usage: %s set <1|0>\n", argv[0]);
    return 1;
  }

  int tmp = atoi(argv[2]);
  _set_opt(netdev, NETOPT_FIXED_HEADER, tmp, "implicit header");
  return 0;
}

int payload_cmd(int argc, char **argv) {
  netdev_t *netdev = (netdev_t *)&sx128x;
  if (argc < 3 || strcmp(argv[1], "set") != 0) {
    printf("usage: %s set <payload length>\n", argv[0]);
    return 1;
  }

  uint16_t tmp = atoi(argv[2]);
  netdev->driver->set(netdev, NETOPT_PDU_SIZE, &tmp, sizeof(tmp));
  printf("Successfully set payload to %i\n", tmp);
  return 0;
}

int get_status_cmd(int argc, char **argv) {
  /* netdev_t *netdev = (netdev_t *)&sx128x; */
  if (argc > 1) {
    printf("usage: %s\n", argv[0]);
    return 1;
  }
  uint8_t status = sx128x_cmd_get_status(&sx128x);
  printf("%#02x\n", status);
  uint8_t cmd_status = ((status & SX128X_STATUS_COMMAND_STATUS_MASK) >> SX128X_STATUS_COMMAND_STATUS_MASK_OFFSET);
  printf("%#02x: ", cmd_status);
  switch (cmd_status) {
    case 1:
      printf("Transceiver successfully processed the command\n");
      break;
    case 2:
      printf("Data are available to host\n");
      break;
    case 3:
      printf("Command time-out\n");
      break;
    case 4:
      printf("Command processing error\n");
      break;
    case 5:
      printf("Failure to execute command\n");
      break;
    case 6:
      printf("Command TX done\n");
      break;
    default:
      printf("Reserved\n");
  }

  uint8_t circ_mode = ((status & SX128X_STATUS_CIRCUIT_MODE_MASK) >> SX128X_STATUS_CIRCUIT_MODE_MASK_OFFSET);
  printf("%#02x: ", circ_mode);
  switch (circ_mode) {
    case 2:
      printf("STDBY_RC\n");
      break;
    case 3:
      printf("STDBY_XOSC\n");
      break;
    case 4:
      printf("FS\n");
      break;
    case 5:
      printf("RX\n");
      break;
    case 6:
      printf("TX\n");
      break;
    default:
      printf("Reserved\n");
  }
  return 0;
}

int get_irq_status_cmd(int argc, char **argv) {
  /* netdev_t *netdev = (netdev_t *)&sx128x; */
  if (argc > 1) {
    printf("usage: %s\n", argv[0]);
    return 1;
  }
  uint16_t status = sx128x_cmd_get_irq_status(&sx128x);
  printf("%#04x\n", status);
  return 0;
}



int radio_operation_cmd(int argc, char **argv) {
  /* netdev_t *netdev = (netdev_t *)&sx128x; */
  if (argc < 2) {
    printf("usage: %s <standby|sleep>\n", argv[0]);
    return 1;
  }

  if (strcmp(argv[1], "standby") == 0) {
    sx128x_cmd_set_standby(&sx128x, 0);
  } else if (strcmp(argv[1], "standby") == 0) {
    sx128x_cmd_set_sleep(&sx128x, 0);
  }
  return 0;
}

static const shell_command_t shell_commands[] = {
    {"setup", "Initialize LoRa modulation settings", lora_setup_cmd},
    {"status", "Get Status of the module", get_status_cmd},
    {"irq_status", "Get IRQ Status of the module", get_irq_status_cmd},
    {"rop", "Access the radio operation commands", radio_operation_cmd},
    {"implicit", "Enable implicit header", implicit_cmd},
    {"crc", "Enable CRC", crc_cmd},
    {"payload", "Set payload length (implicit header)", payload_cmd},
    {"random", "Get random number from sx128x", random_cmd},
    {"syncword", "Get/Set the syncword", syncword_cmd},
    {"rx_timeout", "Set the RX timeout", rx_timeout_cmd},
    {"channel", "Get/Set channel frequency (in Hz)", channel_cmd},
    {"register", "Get/Set value(s) of registers of sx128x", register_cmd},
    {"send", "Send raw payload string", send_cmd},
    {"listen", "Start raw payload listener", listen_cmd},
    {"reset", "Reset the sx128x device", reset_cmd},
    {NULL, NULL, NULL}};

static void _event_cb(netdev_t *dev, netdev_event_t event) {
  if (event == NETDEV_EVENT_ISR) {
    msg_t msg;

    msg.type = MSG_TYPE_ISR;
    msg.content.ptr = dev;

    if (msg_send(&msg, _recv_pid) <= 0) {
      puts("gnrc_netdev: possibly lost interrupt.");
    }
  } else {
    size_t len;
    netdev_lora_rx_info_t packet_info;
    switch (event) {
    case NETDEV_EVENT_RX_STARTED:
      puts("Data reception started");
      break;

    case NETDEV_EVENT_RX_COMPLETE:
      len = dev->driver->recv(dev, NULL, 0, 0);
      dev->driver->recv(dev, message, len, &packet_info);
      /* printf("{Payload: \"%s\" (%d bytes), RSSI: %i, SNR: %i, TOA: %" PRIu32 */
      /*        "}\n", */
      /*        message, (int)len, packet_info.rssi, (int)packet_info.snr, */
      /*        sx128x_get_time_on_air((const sx128x_t *)dev, len)); */
      break;

    case NETDEV_EVENT_TX_COMPLETE:
      sx128x_set_sleep(&sx128x);
      puts("Transmission completed");
      break;

    case NETDEV_EVENT_CAD_DONE:
      break;

    case NETDEV_EVENT_TX_TIMEOUT:
      sx128x_set_sleep(&sx128x);
      break;

    default:
      printf("Unexpected netdev event received: %d\n", event);
      break;
    }
  }
}

void *_recv_thread(void *arg) {
  (void)arg;

  static msg_t _msg_q[SX128X_LORA_MSG_QUEUE];
  msg_init_queue(_msg_q, SX128X_LORA_MSG_QUEUE);

  while (1) {
    msg_t msg;
    msg_receive(&msg);
    if (msg.type == MSG_TYPE_ISR) {
      netdev_t *dev = msg.content.ptr;
      dev->driver->isr(dev);
    } else {
      puts("Unexpected msg type");
    }
  }
}

int main(void) {
  sx128x.params = sx128x_params[0];
  netdev_t *netdev = (netdev_t *)&sx128x;
  netdev->driver = &sx128x_driver;

  if (netdev->driver->init(netdev) < 0) {
    puts("Failed to initialize sx128x device, exiting");
    return 1;
  }

  netdev->event_callback = _event_cb;

  _recv_pid =
      thread_create(stack, sizeof(stack), THREAD_PRIORITY_MAIN - 1,
                    THREAD_CREATE_STACKTEST, _recv_thread, NULL, "recv_thread");

  if (_recv_pid <= KERNEL_PID_UNDEF) {
    puts("Creation of receiver thread failed");
    return 1;
  }

  /* start the shell */
  puts("Initialization successful - starting the shell now");
  char line_buf[SHELL_DEFAULT_BUFSIZE];
  shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);

  return 0;
}
