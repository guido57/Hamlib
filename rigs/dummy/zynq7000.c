/*
*  Hamlib rigctld backend - works with SDR#'s gpredict plugin for example
*  Copyright (c) 2023 by Michael Black W9MDB
*
*
*   This library is free software; you can redistribute it and/or
*   modify it under the terms of the GNU Lesser General Public
*   License as published by the Free Software Foundation; either
*   version 2.1 of the License, or (at your option) any later version.
*
*   This library is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
*   Lesser General Public License for more details.
*
*   You should have received a copy of the GNU Lesser General Public
*   License along with this library; if not, write to the Free Software
*   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <hamlib/rig.h>
#include <serial.h>
#include <misc.h>
#include <unistd.h>

#include "uio_c.h"

#define DEBUG 1
#define DEBUG_TRACE DEBUG_VERBOSE
#define TRUE 1
#define FALSE 0


#define MAXCMDLEN 128
#define MAXXMLLEN 128
#define MAXARGLEN 128
#define MAXBANDWIDTHLEN 4096

//#define DEFAULTPATH "127.0.0.1:4532"

#define ZYNQ7000_VFOS (RIG_VFO_A)

//#define ZYNQ7000_MODES (RIG_MODE_NONE)
#define ZYNQ7000_MODES (RIG_MODE_AM | RIG_MODE_LSB | RIG_MODE_USB)

#define CMDSLEEP 20*1000  /* ms for each command */

// -----------------------------
// UIO devices structs
struct UIO dev_adc_test_switch;
struct UIO dev_am_ssb_switch;
struct DDS dev_dds_lo;
struct DDS dev_dds_bfo;
struct FilterGain dev_if_filter_gain;
struct DecimationRate dev_decimation_rate_iq;

struct zynq7000_priv_data
{
    vfo_t curr_vfo;
    channel_t vfo_a;
    char bandwidths[MAXBANDWIDTHLEN]; /* pipe delimited set */
    int nbandwidths;
    char info[8192];
    ptt_t ptt;
    split_t split;
    rmode_t curr_modeA;
    rmode_t curr_modeB;
    freq_t curr_freqA;
    freq_t curr_freqB;
    pbwidth_t curr_widthA;
    pbwidth_t curr_widthB;
    int has_get_modeA; /* True if this function is available */
    int has_get_bwA; /* True if this function is available */
    int has_set_bwA; /* True if this function is available */
    float powermeter_scale;  /* So we can scale power meter to 0-1 */
    value_t parms[RIG_SETTING_MAX];
    struct ext_list *ext_parms;
};

/*
* check_vfo
* No assumptions
*/
static int check_vfo(vfo_t vfo)
{
    switch (vfo)
    {
    case RIG_VFO_A:
        break;

    case RIG_VFO_TX:
    case RIG_VFO_B:
        break;

    case RIG_VFO_CURR:
        break;                  // will default to A in which_vfo

    default:
        return (FALSE);
    }

    return (TRUE);
}

/*
* read_transaction
* Assumes rig!=NULL, xml!=NULL, xml_len>=MAXXMLLEN
*/
static int read_transaction(RIG *rig, char *xml, int xml_len)
{
    int retval;
    int retry;
    char *delims;
    char *terminator = "\n";
    struct rig_state *rs = &rig->state;

    ENTERFUNC;

    retry = 2;
    delims = "\n";
    xml[0] = 0;

    do
    {
        char tmp_buf[MAXXMLLEN];        // plenty big for expected sdrsharp responses hopefully

        if (retry < 2)
        {
            rig_debug(RIG_DEBUG_WARN, "%s: retry needed? retry=%d\n", __func__, retry);
        }

        int len = read_string(&rs->rigport, (unsigned char *) tmp_buf, sizeof(tmp_buf),
                              delims,
                              strlen(delims), 0, 1);

        if (len > 0) { retry = 3; }

        if (len <= 0)
        {
            rig_debug(RIG_DEBUG_ERR, "%s: read_string error=%d\n", __func__, len);
            continue;
        }

        if (strlen(xml) + strlen(tmp_buf) < xml_len - 1)
        {
            strncat(xml, tmp_buf, xml_len - 1);
        }
        else
        {
            rig_debug(RIG_DEBUG_ERR,
                      "%s: xml buffer overflow!!\nTrying to add len=%d\nTo len=%d\n", __func__,
                      (int)strlen(tmp_buf), (int)strlen(xml));
            RETURNFUNC(-RIG_EPROTO);
        }
    }
    while (retry-- > 0 && strstr(xml, terminator) == NULL);

    if (retry == 0)
    {
        rig_debug(RIG_DEBUG_WARN, "%s: retry timeout\n", __func__);
        RETURNFUNC(-RIG_ETIMEOUT);
    }

    if (strstr(xml, terminator))
    {
//        rig_debug(RIG_DEBUG_TRACE, "%s: got %s\n", __func__, terminator);
        retval = RIG_OK;
    }
    else
    {
        rig_debug(RIG_DEBUG_VERBOSE, "%s: did not get %s\n", __func__, terminator);
        retval = -(101 + RIG_EPROTO);
    }

    RETURNFUNC(retval);
}

/*
* write_transaction
* Assumes rig!=NULL, xml!=NULL, xml_len=total size of xml for response
*/
static int write_transaction(RIG *rig, char *xml, int xml_len)
{

    int try = rig->caps->retry;

    int retval = -RIG_EPROTO;

    struct rig_state *rs = &rig->state;

    ENTERFUNC;

    // This shouldn't ever happen...but just in case
    // We need to avoid an empty write as rigctld replies with blank line
    if (xml_len == 0)
    {
        rig_debug(RIG_DEBUG_ERR, "%s: len==0??\n", __func__);
        RETURNFUNC(retval);
    }

    // appears we can lose sync if we don't clear things out
    // shouldn't be anything for us now anyways
    rig_flush(&rig->state.rigport);

    while (try-- >= 0 && retval != RIG_OK)
        {
            retval = write_block(&rs->rigport, (unsigned char *) xml, strlen(xml));

            if (retval  < 0)
            {
                RETURNFUNC(-RIG_EIO);
            }
        }

    RETURNFUNC(retval);
}
/*
static int zynq7000_transaction(RIG *rig, char *cmd, char *value,
                                int value_len)
{
    char xml[MAXXMLLEN];
    int retry = 3;

    ENTERFUNC;
    ELAPSED1;

    set_transaction_active(rig);

    if (value)
    {
        value[0] = 0;
    }

    do
    {
        int retval;

        if (retry != 3)
        {
            rig_debug(RIG_DEBUG_VERBOSE, "%s: cmd=%s, retry=%d\n", __func__, cmd, retry);
        }

        retval = write_transaction(rig, cmd, strlen(cmd));

        if (retval != RIG_OK)
        {
            rig_debug(RIG_DEBUG_ERR, "%s: write_transaction error=%d\n", __func__, retval);

            // if we get RIG_EIO the socket has probably disappeared
            // so bubble up the error so port can re re-opened
            if (retval == -RIG_EIO) { set_transaction_inactive(rig); RETURNFUNC(retval); }

            hl_usleep(50 * 1000); // 50ms sleep if error
        }

        if (value)
        {
            read_transaction(rig, xml, sizeof(xml));    // this might time out -- that's OK
        }

        if (value) { strncpy(value, xml, value_len); }

    }
    while (((value && strlen(value) == 0))
            && retry--); // we'll do retries if needed

    if (value && strlen(value) == 0)
    {
        rig_debug(RIG_DEBUG_ERR, "%s: no value returned\n", __func__);
        set_transaction_inactive(rig); RETURNFUNC(RIG_EPROTO);
    }

    ELAPSED2;
    set_transaction_inactive(rig);
    RETURNFUNC(RIG_OK);
}
*/

/*
* zynq7000_get_freq
* Assumes rig!=NULL, rig->state.priv!=NULL, freq!=NULL
*/
static int zynq7000_get_freq(RIG *rig, vfo_t vfo, freq_t *freq)
{
    char value[MAXARGLEN];
    struct zynq7000_priv_data *priv = (struct zynq7000_priv_data *) rig->state.priv;

    ENTERFUNC;
    rig_debug(RIG_DEBUG_TRACE, "%s: vfo=%s\n", __func__,
              rig_strvfo(vfo));


    if (check_vfo(vfo) == FALSE)
    {
        rig_debug(RIG_DEBUG_ERR, "%s: unsupported VFO %s\n",
                  __func__, rig_strvfo(vfo));
        RETURNFUNC(-RIG_EINVAL);
    }

    if (vfo == RIG_VFO_CURR)
    {
        vfo = rig->state.current_vfo;
        rig_debug(RIG_DEBUG_TRACE, "%s: get_freq2 vfo=%s\n",
                  __func__, rig_strvfo(vfo));
    }

    //char *cmd = "f\n";
    int retval;

    //GG retval = zynq7000_transaction(rig, cmd, value, sizeof(value));
    *freq = dev_dds_lo.current_freq_hz;
    printf("zynq7000_get_freq: rig model=%s vfo=%d freq=%lf\r\n",rig->caps->model_name, vfo, *freq);

    retval = RIG_OK;

    if (retval != RIG_OK)
    {
        rig_debug(RIG_DEBUG_ERR, "%s: READBMF failed retval=%s\n", __func__,
                  rigerror(retval));
        RETURNFUNC(retval);
    }

    sscanf(value, "%lf", freq);

    if (*freq == 0)
    {
        rig_debug(RIG_DEBUG_ERR, "%s: freq==0??\nvalue=%s\n", __func__,
                  value);
        RETURNFUNC(-RIG_EPROTO);

    }
    else
    {
        rig_debug(RIG_DEBUG_TRACE, "%s: freq=%.0f\n", __func__, *freq);
    }

    if (vfo == RIG_VFO_A)
    {
        priv->curr_freqA = *freq;
    }
    else // future support in zynq7000 maybe?
    {
        priv->curr_freqB = *freq;
    }

    RETURNFUNC(RIG_OK);
}



/*
* zynq7000_open
* Assumes rig!=NULL, rig->state.priv!=NULL
*/
static int zynq7000_open(RIG *rig)
{
    int retval;
    char value[MAXARGLEN];

    ENTERFUNC;

    freq_t freq;
    retval = zynq7000_get_freq(rig, RIG_VFO_CURR, &freq);

    if (retval != RIG_OK)
    {
        rig_debug(RIG_DEBUG_ERR, "%s: zynq7000_get_freq not working!!\n", __func__);
        RETURNFUNC(RIG_EPROTO);
    }

    rig->state.current_vfo = RIG_VFO_A;
    rig_debug(RIG_DEBUG_TRACE, "%s: currvfo=%s value=%s\n", __func__,
              rig_strvfo(rig->state.current_vfo), value);

    RETURNFUNC(retval);
}

/*
* zynq7000_close
* Assumes rig!=NULL
*/
static int zynq7000_close(RIG *rig)
{
    ENTERFUNC;

    RETURNFUNC(RIG_OK);
}

/*
* zynq7000_cleanup
* Assumes rig!=NULL, rig->state.priv!=NULL
*/
static int zynq7000_cleanup(RIG *rig)
{
    struct zynq7000_priv_data *priv;

    rig_debug(RIG_DEBUG_TRACE, "%s\n", __func__);

    if (!rig)
    {
        RETURNFUNC2(-RIG_EINVAL);
    }

    priv = (struct zynq7000_priv_data *)rig->state.priv;

    free(priv->ext_parms);
    free(rig->state.priv);

    rig->state.priv = NULL;

    // we really don't need to free this up as it's only done once
    // was causing problem when cleanup was followed by rig_open
    // model_zynq7000 was not getting refilled
    // if we can figure out that one we can re-enable this
#if 0
    int i;

    for (i = 0; modeMap[i].mode_hamlib != 0; ++i)
    {
        if (modeMap[i].mode_zynq7000)
        {
            free(modeMap[i].mode_zynq7000);
            modeMap[i].mode_zynq7000 = NULL;
            modeMap[i].mode_hamlib = 0;
        }

    }

#endif

    RETURNFUNC2(RIG_OK);
}


/*
* zynq7000_set_freq
* assumes rig!=NULL, rig->state.priv!=NULL
*/
static int zynq7000_set_freq(RIG *rig, vfo_t vfo, freq_t freq)
{
    //int retval;
    char cmd[MAXARGLEN];
    //char value[1024];

    //struct zynq7000_priv_data *priv = (struct zynq7000_priv_data *) rig->state.priv;

    rig_debug(RIG_DEBUG_TRACE, "%s\n", __func__);
    rig_debug(RIG_DEBUG_TRACE, "%s: vfo=%s freq=%.0f\n", __func__,
              rig_strvfo(vfo), freq);

    if (check_vfo(vfo) == FALSE)
    {
        rig_debug(RIG_DEBUG_ERR, "%s: unsupported VFO %s\n",
                  __func__, rig_strvfo(vfo));
        RETURNFUNC2(-RIG_EINVAL);
    }

    if (vfo == RIG_VFO_CURR)
    {
        vfo = rig->state.current_vfo;
    }

    SNPRINTF(cmd, sizeof(cmd), "F %.0lf\n", freq);

    //retval = zynq7000_transaction(rig, cmd, value, sizeof(value));
    rig_debug(RIG_DEBUG_TRACE,"zynq7000_set_freq: rig model=%s vfo=%d freq=%f\r\n",rig->caps->model_name, vfo, freq);
    int ret = DDS_SetFreq(&dev_dds_lo, freq);
    if (ret <= 0)
    {
        RETURNFUNC2(ret);
    }

    int freqBFO = rig->state.current_width ;
    int freqLO = freq;

    switch(rig->state.current_mode){
        case RIG_MODE_USB:
            freqLO = freq + freqBFO;
            break;
        case RIG_MODE_LSB:
            freqLO = freq - freqBFO;
            break;
    }

    DDS_SetFreq(&dev_dds_bfo, freqBFO);
    rig_debug(RIG_DEBUG_TRACE,"zynq7000_set_freq: BFO frequency=%d\r\n", freqBFO);

    DDS_SetFreq(&dev_dds_lo, freqLO);
    rig_debug(RIG_DEBUG_TRACE,"zynq7000_set_freq: LO frequency=%d\r\n", freqLO);

    //sscanf(value, "RPRT %d", &retval);
    //RETURNFUNC2(retval);
    RETURNFUNC(RIG_OK);
}

static int zynq7000_set_mode(RIG *rig, vfo_t vfo, rmode_t mode, pbwidth_t width)
{
    struct zynq7000_priv_data *priv = (struct dummy_priv_data *)rig->state.priv;
    //channel_t *curr = priv->curr;
    char buf[16];

    ENTERFUNC;
    usleep(CMDSLEEP);
    sprintf_freq(buf, sizeof(buf), width);
    rig_debug(RIG_DEBUG_VERBOSE, "%s called: %s %s %s\n", __func__,
              rig_strvfo(vfo), rig_strrmode(mode), buf);

/*
    vfo = vfo_fixup(rig, vfo, rig->state.cache.split);

    if (width == RIG_PASSBAND_NOCHANGE)
    {
            switch (vfo)
            {
            case RIG_VFO_MAIN:
            case RIG_VFO_A: width = priv->vfo_a.width; break;

            case RIG_VFO_SUB:
            case RIG_VFO_B: width = priv->vfo_b.width; break;

            case RIG_VFO_C: width = priv->vfo_c.width; break;
            }
    }

    switch (vfo)
    {
    case RIG_VFO_MAIN:
    case RIG_VFO_A: priv->vfo_a.mode = mode; priv->vfo_a.width = width; break;

    case RIG_VFO_SUB:
    case RIG_VFO_B: priv->vfo_b.mode = mode; priv->vfo_b.width = width; break;

    case RIG_VFO_C: priv->vfo_c.mode = mode; priv->vfo_c.width = width; break;

    default:
            rig_debug(RIG_DEBUG_ERR, "%s: unknown VFO=%s\n", __func__, rig_strvfo(vfo));
            RETURNFUNC(-RIG_EINVAL);
    }

    vfo = vfo_fixup(rig, vfo, rig->state.cache.split);

    if (RIG_PASSBAND_NOCHANGE == width) { RETURNFUNC(RIG_OK); }

    if (width == RIG_PASSBAND_NORMAL)
    {
            width = curr->width = rig_passband_normal(rig, mode);
    }

    switch (vfo)
    {
    case RIG_VFO_A: priv->vfo_a.width = width; break;

    case RIG_VFO_B: priv->vfo_b.width = width; break;

    case RIG_VFO_C: priv->vfo_c.width = width; break;
    }
*/
    // Set AM USB or LSB
    switch(mode){
    case RIG_MODE_AM:
        AMSSBSwitch_SetAM(&dev_am_ssb_switch);
            break;
    case RIG_MODE_USB:
        AMSSBSwitch_SetUSB(&dev_am_ssb_switch);
        break;
    case RIG_MODE_LSB:
        AMSSBSwitch_SetLSB(&dev_am_ssb_switch);
        break;
    }

    // set bandwidth
    if(width <=3000){
        width = 2000;
        DecimationRate_SetBandwidth(&dev_decimation_rate_iq,"2");
    }else if(width < 5000){
        width = 4000;
        DecimationRate_SetBandwidth(&dev_decimation_rate_iq,"4");
    }else if(width < 12000){
        width = 8000;
        DecimationRate_SetBandwidth(&dev_decimation_rate_iq,"8");
    }else if(width < 24000){
        width = 16000;
        DecimationRate_SetBandwidth(&dev_decimation_rate_iq,"16");
    }else{
        width = 30000;
        DecimationRate_SetBandwidth(&dev_decimation_rate_iq,"30");
    }
    priv->vfo_a.mode = mode;
    priv->vfo_a.width = width;

    RETURNFUNC(RIG_OK);
}


static int zynq7000_get_mode(RIG *rig, vfo_t vfo, rmode_t *mode, pbwidth_t *width)
{
    struct zynq7000_priv_data *priv = (struct zynq7000_priv_data *)rig->state.priv;

    ENTERFUNC;
    usleep(CMDSLEEP);
    rig_debug(RIG_DEBUG_VERBOSE, "%s called: %s\n", __func__, rig_strvfo(vfo));

    *mode = priv->vfo_a.mode;
    *width = priv->vfo_a.width;

    RETURNFUNC(RIG_OK);
}



/*
* zynq7000_get_vfo
* assumes rig!=NULL, vfo != NULL
*/
static int zynq7000_get_vfo(RIG *rig, vfo_t *vfo)
{
    ENTERFUNC;

    *vfo = RIG_VFO_A;

    RETURNFUNC(RIG_OK);
}

/*
* zynq7000_init
* Assumes rig!=NULL
*/
static int zynq7000_init(RIG *rig)
{
    struct zynq7000_priv_data *priv;

    ENTERFUNC;
    rig_debug(RIG_DEBUG_TRACE, "%s version %s\n", __func__, rig->caps->version);

    rig->state.priv  = (struct zynq7000_priv_data *)calloc(1, sizeof(
                                                                  struct zynq7000_priv_data));

    if (!rig->state.priv)
    {
        RETURNFUNC(-RIG_ENOMEM);
    }

    priv = rig->state.priv;

    memset(priv, 0, sizeof(struct zynq7000_priv_data));
    memset(priv->parms, 0, RIG_SETTING_MAX * sizeof(value_t));

    // set bandwidths
    priv->nbandwidths = 4;
    strcpy(priv->bandwidths, "2000|4000|8000|16000|30000");

    /*
     * set arbitrary initial status
     */
    rig->state.current_vfo = RIG_VFO_A;
    priv->split = 0;
    priv->ptt = 0;
    priv->curr_modeA = -1;
    priv->curr_modeB = -1;
    //priv->curr_widthA = -1;
    // set initial bandwith
    priv->curr_widthA = 4000;
    priv->curr_widthB = -1;
    priv->curr_freqA = MHz(16);
    priv->curr_vfo = 1;
    priv->vfo_a.split = RIG_SPLIT_OFF;

    // Init UIO devices
    strcpy(dev_adc_test_switch.devuio, DEV_ADC_TEST_SWITCH);
    ADCTestSwitch_Init(&dev_adc_test_switch);

    strcpy(dev_am_ssb_switch.devuio, DEV_AM_SSB);
    AMSSBSwitch_Init(&dev_am_ssb_switch);

    strcpy(dev_dds_lo.uio.devuio, DEV_DDS_LO);
    dev_dds_lo.b_phase_width = 26;
    dev_dds_lo.master_clock_hz = 64000000;
    DDS_Init(&dev_dds_lo);

    strcpy(dev_dds_bfo.uio.devuio, DEV_DDS_BFO);
    dev_dds_bfo.b_phase_width = 26;
    dev_dds_bfo.master_clock_hz = 64000000;
    DDS_Init(&dev_dds_bfo);

    strcpy(dev_if_filter_gain.uio.devuio, DEV_IF_GAIN);
    FilterGain_init(&dev_if_filter_gain);

    strcpy(dev_decimation_rate_iq.uio.devuio, DEV_DEC_RATE_IQ);
    DecimationRate_Init(&dev_decimation_rate_iq);

    // set the initial mode to USB
    AMSSBSwitch_SetUSB(&dev_am_ssb_switch);

    // set the bandwidth to 4KHz
    DecimationRate_SetBandwidth(&dev_decimation_rate_iq, "4");

    // set the BFO to 4KHz
    DDS_SetFreq(&dev_dds_bfo, 4000);


    // set the initial frequency
    zynq7000_set_freq(rig, priv->curr_vfo, priv->curr_freqA);

    if (!rig->caps)
    {
        RETURNFUNC(-RIG_EINVAL);
    }

    //strncpy(rig->state.rigport.pathname, DEFAULTPATH,
    //        sizeof(rig->state.rigport.pathname));

    RETURNFUNC(RIG_OK);
}


struct rig_caps zynq7000_caps =
{
    RIG_MODEL(RIG_MODEL_ZYNQ7000),
    .model_name = "ZYNQ7000",
    .mfg_name = "IW5ALZ",
    .version = "20230703.0",
    .copyright = "LGPL",
    .status = RIG_STATUS_STABLE,
    .rig_type = RIG_TYPE_COMPUTER,
    //.targetable_vfo =  RIG_TARGETABLE_FREQ | RIG_TARGETABLE_MODE,
    .ptt_type = RIG_PTT_NONE,
    .port_type = RIG_PORT_NONE,
    .write_delay = 0,
    .post_write_delay = 0,
    .timeout = 1000,
    .retry = 2,

    .filters =  {
        {RIG_MODE_ALL, RIG_FLT_ANY},
        RIG_FLT_END
    },

    .rx_range_list1 = {{
            .startf = kHz(1), .endf = GHz(10), .modes = ZYNQ7000_MODES,
            .low_power = -1, .high_power = -1, ZYNQ7000_VFOS, RIG_ANT_1
        },
        RIG_FRNG_END,
    },
    .tx_range_list1 = {RIG_FRNG_END,},
    .rx_range_list2 = {{
            .startf = kHz(1), .endf = GHz(10), .modes = ZYNQ7000_MODES,
            .low_power = -1, .high_power = -1, ZYNQ7000_VFOS, RIG_ANT_1
        },
        RIG_FRNG_END,
    },
    .tx_range_list2 = {RIG_FRNG_END,},
    .tuning_steps =  { {ZYNQ7000_MODES, 1}, {ZYNQ7000_MODES, RIG_TS_ANY}, RIG_TS_END, },
    .priv = NULL,               /* priv */

    .rig_init = zynq7000_init,
    .rig_open = zynq7000_open,
    .rig_close = zynq7000_close,
    .rig_cleanup = zynq7000_cleanup,

    .get_vfo = zynq7000_get_vfo,
    .set_freq = zynq7000_set_freq,
    .get_freq = zynq7000_get_freq,

    .get_mode = zynq7000_get_mode,
    .set_mode = zynq7000_set_mode,

    .hamlib_check_rig_caps = HAMLIB_CHECK_RIG_CAPS
};
