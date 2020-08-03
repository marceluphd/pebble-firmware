#include <stdlib.h>
#include <stdio.h>
#include <zephyr.h>
#include <at_cmd.h>
#include <string.h>
#include "modem_helper.h"


static bool isLeap(uint32_t year) {
    return (((year % 4) == 0) && (((year % 100) != 0) || ((year % 400) == 0)));
}

const char *iotex_modem_get_imei() {

    int err;
    enum at_cmd_state at_state;
    static char imei_buf[MODEM_IMEI_LEN + 5];

    if ((err = at_cmd_write("AT+CGSN", imei_buf, sizeof(imei_buf), &at_state))) {
        printk("Error when trying to do at_cmd_write: %d, at_state: %d", err, at_state);
        return "Unknown";
    }

    return imei_buf;
}

int iotex_model_get_signal_quality() {

    enum at_cmd_state at_state;
    static char snr_ack[32], snr[4];

    int err = at_cmd_write("AT+CESQ", snr_ack, 32, &at_state);

    if (err) {
        printk("Error when trying to do at_cmd_write: %d, at_state: %d", err, at_state);
    }

    snprintf(snr, sizeof(snr), "%s", &snr_ack[25]);
    return atoi(snr);
}


const char *iotex_modem_get_clock(iotex_st_timestamp *stamp) {

    double epoch;
    enum at_cmd_state at_state;
    uint32_t YY, MM, DD, hh, mm, ss;

    static char cclk_r_buf[TIMESTAMP_STR_LEN];
    static char epoch_buf[TIMESTAMP_STR_LEN];
    static int daysPerMonth[] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

    int err = at_cmd_write("AT+CCLK?", cclk_r_buf, sizeof(cclk_r_buf), &at_state);

    if (err) {
        printk("Error when trying to do at_cmd_write: %d, at_state: %d", err, at_state);
    }

    printk("AT CMD Modem time is:%s\n", cclk_r_buf);

    // num of years since 1900, the formula works only for 2xxx
    YY = (cclk_r_buf[8] - '0') * 10 + (cclk_r_buf[9] - '0') + 100;
    MM = (cclk_r_buf[11] - '0') * 10 + (cclk_r_buf[12] - '0');
    DD = (cclk_r_buf[14] - '0') * 10 + (cclk_r_buf[15] - '0');
    hh = (cclk_r_buf[17] - '0') * 10 + (cclk_r_buf[18] - '0');
    mm = (cclk_r_buf[20] - '0') * 10 + (cclk_r_buf[21] - '0');
    ss = (cclk_r_buf[23] - '0') * 10 + (cclk_r_buf[24] - '0');

    if (isLeap(YY + 1900)) {
        daysPerMonth[2] = 29;
    }

    // accumulate
    for (int i = 1; i <= 12; i++) {
        daysPerMonth[i] += daysPerMonth[i - 1];
    }

    epoch  = ss + mm * 60 + hh * 3600 + (daysPerMonth[ MM - 1] + DD - 1) * 86400 +
             (YY - 70) * 31536000L + ((YY - 69) / 4) * 86400L -
             ((YY - 1) / 100) * 86400L + ((YY + 299) / 400) * 86400L;

    snprintf(epoch_buf, sizeof(epoch_buf), "%.0f", epoch);
    printk("UTC epoch %s\n", epoch_buf);

    if (stamp) {
        snprintf(stamp->data, sizeof(stamp->data), "%.0f", epoch);
        return stamp->data;
    }

    return epoch_buf;
}
