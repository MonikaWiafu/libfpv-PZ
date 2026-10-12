/*
 * libfpv - Betaflight-side glue.
 *
 * Compiled together with the Betaflight sources (same include paths and
 * defines). Everything here must only be called from the simulation thread,
 * the same thread that runs Betaflight's scheduler.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "platform.h"

#include "cli/cli.h"
#include "config/config.h"
#include "drivers/serial.h"
#include "fc/controlrate_profile.h"
#include "fc/core.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"
#include "flight/failsafe.h"
#include "flight/mixer.h"
#include "io/beeper.h"
#include "pg/pg.h"
#include "config/config_eeprom.h"
#include "drivers/system.h"

#include "drivers/serial_ws.h"

#include "bf_bridge.h"

/* ---------------------------------------------------------------------------
 * In-memory serial port used to talk to Betaflight's CLI from libfpv.
 * ------------------------------------------------------------------------- */

typedef struct {
    serialPort_t port;
    const char *in;
    size_t inLen;
    size_t inPos;
    char *out;
    size_t outLen;
    size_t outCap;
} memPort_t;

static uint8_t memPortDummyBuffer[1];

static void memWrite(serialPort_t *instance, uint8_t ch)
{
    memPort_t *p = (memPort_t *)instance;
    /* strip the CLI's STX/ETX flow control bytes */
    if (ch == 0x2 || ch == 0x3) {
        return;
    }
    if (p->outLen + 2 > p->outCap) {
        size_t cap = p->outCap ? p->outCap * 2 : 4096;
        char *grown = (char *)realloc(p->out, cap);
        if (!grown) {
            return;
        }
        p->out = grown;
        p->outCap = cap;
    }
    p->out[p->outLen++] = (char)ch;
    p->out[p->outLen] = 0;
}

static uint32_t memRxWaiting(const serialPort_t *instance)
{
    const memPort_t *p = (const memPort_t *)instance;
    return (uint32_t)(p->inLen - p->inPos);
}

static uint32_t memTxFree(const serialPort_t *instance)
{
    UNUSED(instance);
    return 0x10000;
}

static uint8_t memRead(serialPort_t *instance)
{
    memPort_t *p = (memPort_t *)instance;
    return p->inPos < p->inLen ? (uint8_t)p->in[p->inPos++] : 0;
}

static bool memTxEmpty(const serialPort_t *instance)
{
    UNUSED(instance);
    return true;
}

static const struct serialPortVTable memVTable = {
    .serialWrite = memWrite,
    .serialTotalRxWaiting = memRxWaiting,
    .serialTotalTxFree = memTxFree,
    .serialRead = memRead,
    .serialSetBaudRate = NULL,
    .isSerialTransmitBufferEmpty = memTxEmpty,
    .setMode = NULL,
    .setCtrlLineStateCb = NULL,
    .setBaudRateCb = NULL,
    .writeBuf = NULL,
    .beginWrite = NULL,
    .endWrite = NULL,
};

int bfb_cli_run(const char *script, char **output)
{
    *output = NULL;
    if (cliMode) {
        /* the Configurator currently owns the CLI */
        return BFB_CLI_BUSY;
    }

    /* every line must be terminated, then ETX ends the session */
    size_t len = strlen(script);
    char *in = (char *)malloc(len + 3);
    if (!in) {
        return BFB_CLI_ERROR;
    }
    memcpy(in, script, len);
    in[len] = '\n';
    in[len + 1] = 0x3;
    in[len + 2] = 0;

    memPort_t port;
    memset(&port, 0, sizeof(port));
    port.port.vTable = &memVTable;
    port.port.rxBuffer = memPortDummyBuffer;
    port.port.txBuffer = memPortDummyBuffer;
    port.port.rxBufferSize = 1;
    port.port.txBufferSize = 1;
    port.port.identifier = -1;
    port.in = in;
    port.inLen = len + 2;

    cliEnter(&port.port, false);
    while (cliMode && port.inPos < port.inLen) {
        cliProcess();
    }
    free(in);

    if (!port.out) {
        port.out = (char *)calloc(1, 1);
    }
    *output = port.out;
    return cliMode ? BFB_CLI_ERROR : BFB_CLI_OK;
}

void bfb_free(void *p)
{
    free(p);
}

/* ---------------------------------------------------------------------------
 * State accessors
 * ------------------------------------------------------------------------- */

bool bfb_is_armed(void)
{
    return ARMING_FLAG(ARMED) != 0;
}

uint32_t bfb_arming_disable_flags(void)
{
    return (uint32_t)getArmingDisableFlags();
}

const char *bfb_arming_disable_flag_name(int index)
{
    return getArmingDisableFlagName((armingDisableFlags_e)(1u << index));
}

int bfb_arming_disable_flag_count(void)
{
    return ARMING_DISABLE_FLAGS_COUNT;
}

bool bfb_failsafe_active(void)
{
    return failsafeIsActive();
}

uint32_t bfb_flight_mode_flags(void)
{
    return flightModeFlags;
}

bool bfb_config_dirty(void)
{
    return isConfigDirty();
}

void bfb_write_eeprom(void)
{
    writeEEPROM();
}

int bfb_cli_mode(void)
{
    return cliMode ? 1 : 0;
}

bfbRates_t bfb_get_rates(void)
{
    bfbRates_t r;
    memset(&r, 0, sizeof(r));
    const controlRateConfig_t *c = controlRateProfiles(getCurrentControlRateProfileIndex());
    r.ratesType = c->rates_type;
    for (int axis = 0; axis < 3; axis++) {
        r.rcRate[axis] = c->rcRates[axis];
        r.rate[axis] = c->rates[axis];
        r.expo[axis] = c->rcExpo[axis];
        r.rateLimit[axis] = c->rate_limit[axis];
    }
    r.profileIndex = getCurrentControlRateProfileIndex();
    return r;
}

/* EEPROM_FILENAME and loadEEPROMFromFile() live in SimITL's target.c */
extern char *EEPROM_FILENAME;
extern bool loadEEPROMFromFile(void);

int bfb_switch_eeprom(const char *path)
{
    if (isConfigDirty()) {
        writeEEPROM();  /* keep the previous quad's pending changes */
    }
    FILE *probe = fopen(path, "rb");
    int existed = probe != NULL;
    if (probe) {
        fclose(probe);
    }
    size_t n = strlen(path);
    char *name = (char *)malloc(n + 1);
    if (!name) {
        return -1;
    }
    memcpy(name, path, n + 1);
    EEPROM_FILENAME = name; /* previous buffer intentionally kept: SimITL owns it */

    initEEPROM();
    if (!existed) {
        /* a new file was created from the previous quad's memory image */
        resetEEPROM();
    }
    ensureEEPROMStructureIsValid();
    if (!readEEPROM()) {
        resetEEPROM();
    }
    return existed;
}

void bfb_reboot(void)
{
    systemReset();
}

int bfb_reboot_required(void)
{
    return (getArmingDisableFlags() & ARMING_DISABLED_REBOOT_REQUIRED) != 0;
}

void bfb_ws_set_enabled(int enabled)
{
    wsSetEnabled(enabled);
}

int bfb_yaw_motors_reversed(void)
{
    return mixerConfig()->yaw_motors_reversed ? 1 : 0;
}

int bfb_calibrating(void)
{
    return (getArmingDisableFlags() & ARMING_DISABLED_CALIBRATING) != 0;
}

int bfb_settling(void)
{
    return (getArmingDisableFlags() &
            (ARMING_DISABLED_CALIBRATING | ARMING_DISABLED_RX_FAILSAFE | ARMING_DISABLED_BOOT_GRACE_TIME |
             ARMING_DISABLED_FAILSAFE)) != 0;
}

void bfb_ws_set_base_port(int basePort)
{
    wsSetBasePort(basePort);
}

int bfb_ws_base_port(void)
{
    return wsGetBasePort();
}

int bfb_ws_client_count(void)
{
    return wsGetClientCount();
}
