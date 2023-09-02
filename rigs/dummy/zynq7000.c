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

#define NB_CHAN 22      /* see caps->chan_list */

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

static int zynq7000_get_freq(RIG *rig, vfo_t vfo, freq_t *freq);

// -----------------------------
// UIO devices structs
struct AD9851 dev_ad9851;
struct UIO dev_adc_test_switch;
struct UIO dev_am_ssb_switch;
struct DDS dev_dds_lo;
struct DDS dev_dds_bfo;
struct FilterGain dev_if_filter_gain;
struct DecimationRate dev_decimation_rate_iq;

struct zynq7000_priv_data
{
    vfo_t curr_vfo;
    vfo_t last_vfo;/* VFO A or VFO B, when in MEM mode */
    char bandwidths[MAXBANDWIDTHLEN]; /* pipe delimited set */
    int nbandwidths;
    char info[8192];

    split_t split;
    vfo_t tx_vfo;
    ptt_t ptt;

    int trn;
    channel_t *curr;    /* points to vfo_a, vfo_b or mem[] */

    // we're trying to emulate all sorts of vfo possibilities so this looks redundant
    channel_t vfo_a;
    channel_t vfo_b;
    channel_t vfo_c;
    channel_t vfo_maina;
    channel_t vfo_mainb;
    channel_t vfo_suba;
    channel_t vfo_subb;
    channel_t mem[NB_CHAN];

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
    strcpy(dev_ad9851.uio.devuio, DEV_AD9851);
    AD9851_Init(&dev_ad9851,180000000.0);

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
    //AMSSBSwitch_SetUSB(&dev_am_ssb_switch);

    // set the bandwidth to 4KHz
    //DecimationRate_SetBandwidth(&dev_decimation_rate_iq, "4");

    // set the BFO to 4KHz
    //DDS_SetFreq(&dev_dds_bfo, 4000);


    // set the initial frequency
    //zynq7000_set_freq(rig, priv->curr_vfo, priv->curr_freqA);

    if (!rig->caps)
    {
            RETURNFUNC(-RIG_EINVAL);
    }

    //strncpy(rig->state.rigport.pathname, DEFAULTPATH,
    //        sizeof(rig->state.rigport.pathname));

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
* zynq7000_get_freq
*/
static int zynq7000_get_freq(RIG *rig, vfo_t vfo, freq_t *freq)
{
    ENTERFUNC;
    int ptt = AD9851_GetOnOff(&dev_ad9851);
    if(ptt == 0){
        // -------------------- RX mode --------------------------------------------------
        // Get freq according to AM/USB/LSB, dds_lo and dds_bfo
        double freq_LO = DDS_GetFreq(&dev_dds_lo);
        double freq_BFO = DDS_GetFreq(&dev_dds_bfo);
        int amlsbusb = AMSSBSwitch_Get(&dev_am_ssb_switch);
        switch(amlsbusb){
        case 0:
            // USB
            *freq = freq_LO - freq_BFO;
            break;
        case 1:
            // LSB
            *freq = freq_LO + freq_BFO;
            break;
        case 2:
            // AM
            *freq = freq_LO;
            break;
        }

        printf("zynq7000_get_freq: freq=%lf freq_LO=%lf freq_BFO=%lf\r\n", *freq, freq_LO, freq_BFO);

    }else if(ptt == 1){
        // -------------------- TX mode --------------------------------------------------
        *freq = AD9851_GetFreq(&dev_ad9851);
        printf("zynq7000_get_freq in TX: rig model=%s vfo=%d freq=%lf\r\n",rig->caps->model_name, vfo, *freq);
    }

    RETURNFUNC(RIG_OK);
}


/*
* zynq7000_set_freq
* in RX it sets dds_lo according to AM/USB/LSB and dds_bfo
* in TX uses AD9851
*/
static int zynq7000_set_freq(RIG *rig, vfo_t vfo, freq_t freq)
{
    struct timeval stop, start;
    gettimeofday(&start, NULL);
    for(int i=0; i<1;i++){

    int ptt = AD9851_GetOnOff(&dev_ad9851);

    if(ptt == 0){
        // -------------------- RX mode --------------------------------------------------
        // Turn off AD9851 output
        AD9851_SetOff(&dev_ad9851);

        // Set dds_lo and dds_bfo according to AM/USB/LSB
        double freq_BFO = DDS_GetFreq(&dev_dds_bfo);
        double freqLO = freq;                     // round the conversion from double to int

        int amlsbusb = AMSSBSwitch_Get(&dev_am_ssb_switch);
        switch(amlsbusb){
            case 0: // USB
                freqLO = freq + freq_BFO;
                break;
            case 1: // LSB
                freqLO = freq - freq_BFO;
                break;
        }
        int freqLO_int = 0.5 + freqLO;
        DDS_SetFreq(&dev_dds_lo, freqLO_int);
        //printf("zynq7000_set_freq in RX: rig model=%s vfo=%d freq=%lf freqLO=%lf freqLO_int=%d freqBFO=%lf\r\n",rig->caps->model_name, vfo, freq, freqLO, freqLO_int, freq_BFO);

    }else if(ptt == 1){
        // -------------------- TX mode --------------------------------------------------
        // Turn on AD9851 output
        AD9851_SetOn(&dev_ad9851);

        AD9851_SetFreq(&dev_ad9851, freq);
        //printf("zynq7000_set_freq in TX: rig model=%s vfo=%d freq=%lf\r\n",rig->caps->model_name, vfo, freq);
    }

    } // end for

    gettimeofday(&stop, NULL);
    printf("zynq7000_set_mode took %lu us\n", (stop.tv_sec - start.tv_sec) * 1000000 + stop.tv_usec - start.tv_usec);
    RETURNFUNC(RIG_OK);
}

static int zynq7000_set_mode(RIG *rig, vfo_t vfo, rmode_t mode, pbwidth_t width)
{
    //struct zynq7000_priv_data *priv = (struct dummy_priv_data *)rig->state.priv;
    printf("mode=%llu width=%ld\r\n", mode,width);

    ENTERFUNC;
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
    //priv->vfo_a.mode = mode;
    //priv->vfo_a.width = width;
    if(mode == RIG_MODE_AM)
        DDS_SetFreq(&dev_dds_bfo, 0);
    else
        DDS_SetFreq(&dev_dds_bfo, width);

    RETURNFUNC(RIG_OK);
}

static int zynq7000_get_mode(RIG *rig, vfo_t vfo, rmode_t *mode, pbwidth_t *width)
{
    struct zynq7000_priv_data *priv = (struct zynq7000_priv_data *)rig->state.priv;

    ENTERFUNC;
    usleep(CMDSLEEP);
    rig_debug(RIG_DEBUG_VERBOSE, "%s called: %s\n", __func__, rig_strvfo(vfo));

    //*mode = priv->vfo_a.mode;
    //*width = priv->vfo_a.width;

    int amlsbusb = AMSSBSwitch_Get(&dev_am_ssb_switch);
   switch(amlsbusb){
    case 0:
        *mode = RIG_MODE_USB;
        break;
    case 1:
        *mode = RIG_MODE_LSB;
        break;
    case 2:
        *mode = RIG_MODE_AM;
        break;
    }

    char * bw = DecimationRate_GetBandwith(&dev_decimation_rate_iq);
    printf("bw=%s\r\n", bw);
    *width = 1000L * atol(bw);

    RETURNFUNC(RIG_OK);
}


static int zynq7000_set_level(RIG *rig, vfo_t vfo, setting_t level, value_t val)
{

    switch(level){

    case RIG_LEVEL_AF:
        // Set the AF level at a value from 0.0 to 1.0
        printf("zynq7000_set_level RIG=%s VFO=%d level(AF)=%d level_val=%f\r\n",rig->caps->model_name, level, val.f);
        u_int8_t level_0_255 = 255 * val.f;
        AD9851_SetAmplitude_0_255(&dev_ad9851, level_0_255);
        break;

    default:
        printf("zynq7000_set_level RIG=%s VFO=%d level %d not implemented yet!\r\n",rig->caps->model_name, level);
        break;
    }

    RETURNFUNC(RIG_OK);
}


static int zynq7000_get_level(RIG *rig, vfo_t vfo, setting_t level, value_t *val)
{
    u_int8_t level_0_255;

    switch(level){
        case RIG_LEVEL_AF:
            // Get the AF level as a value from 0.0 to 1.0
            AD9851_GetAmplitude_0_255(&dev_ad9851, & level_0_255);
            val->f = level_0_255 / 255.0;
            printf("zynq7000_get_level RIG=%s VFO=%d level(AF)=%d level_val=%f\r\n",rig->caps->model_name, level, val->f);
            break;
        default:
            printf("zynq7000_get_level RIG=%s VFO=%d level %d not implemented yet!\r\n",rig->caps->model_name, level);
            break;
    }

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

    //printf("zynq7000_get_vfo: rig name=%s vfo=%d \r\n",rig->caps->model_name, vfo);
    RETURNFUNC(RIG_OK);
}

static int zynq7000_set_vfo(RIG *rig, vfo_t vfo)
{

    printf("zynq7000_set_vfo: rig name=%s vfo=%d \r\n",rig->caps->model_name, vfo);

    struct zynq7000_priv_data *priv = (struct zynq7000_priv_data *)rig->state.priv;
    channel_t *curr = priv->curr;

    ENTERFUNC;
    usleep(CMDSLEEP);
    rig_debug(RIG_DEBUG_VERBOSE, "%s called: %s\n", __func__, rig_strvfo(vfo));

    if (vfo == RIG_VFO_CURR) { vfo = rig->state.current_vfo; }

    priv->last_vfo = priv->curr_vfo;
    priv->curr_vfo = vfo;

    switch (vfo)
    {
    case RIG_VFO_VFO: /* FIXME */

    case RIG_VFO_RX:
    case RIG_VFO_MAIN: priv->curr = &priv->vfo_a; break;

    case RIG_VFO_MAIN_A: priv->curr = &priv->vfo_maina; break;

    case RIG_VFO_MAIN_B: priv->curr = &priv->vfo_mainb; break;

    case RIG_VFO_A: priv->curr = &priv->vfo_a; break;

    case RIG_VFO_SUB: priv->curr = &priv->vfo_b; break;

    case RIG_VFO_SUB_A: priv->curr = &priv->vfo_suba; break;

    case RIG_VFO_SUB_B: priv->curr = &priv->vfo_subb; break;

    case RIG_VFO_B: priv->curr = &priv->vfo_b; break;

    case RIG_VFO_C: priv->curr = &priv->vfo_c; break;

    case RIG_VFO_MEM:
        if (curr->channel_num >= 0 && curr->channel_num < NB_CHAN)
        {
                priv->curr = &priv->mem[curr->channel_num];
                break;
        }

    case RIG_VFO_TX:
        if (priv->tx_vfo == RIG_VFO_A) { priv->curr = &priv->vfo_a; }
        else if (priv->tx_vfo == RIG_VFO_B) { priv->curr = &priv->vfo_b; }
        else if (priv->tx_vfo == RIG_VFO_MEM) { priv->curr = &priv->mem[curr->channel_num]; }
        else { priv->curr = &priv->vfo_a; }

        break;

    default:
        rig_debug(RIG_DEBUG_VERBOSE, "%s unknown vfo: %s\n", __func__,
                  rig_strvfo(vfo));
        RETURNFUNC(-RIG_EINVAL);
    }

    rig->state.current_vfo = vfo;

    RETURNFUNC(RIG_OK);
}


static int zynq7000_set_ptt(RIG *rig, vfo_t vfo, ptt_t ptt)
{
    struct zynq7000_priv_data *priv = (struct zynq7000_priv_data *)rig->state.priv;

    ENTERFUNC;
    priv->ptt = ptt;

    if(ptt == 1)
        AD9851_SetOn(&dev_ad9851);
    else
        AD9851_SetOff(&dev_ad9851);

    printf("zynq7000_set_ptt: rig name=%s vfo=%d ptt=%d\r\n",rig->caps->model_name, vfo,ptt);

    RETURNFUNC(RIG_OK);
}

static int zynq7000_get_ptt(RIG *rig, vfo_t vfo, ptt_t *ptt)
{
    struct zynq7000_priv_data *priv = (struct zynq7000_priv_data *)rig->state.priv;
    int rc;
    int status = 0;

    ENTERFUNC;

    *ptt = AD9851_GetOnOff(&dev_ad9851);

    printf("zynq7000_get_ptt: rig name=%s vfo=%d ptt=%d\r\n",rig->caps->model_name, vfo, *ptt);

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
    .ptt_type = RIG_PTT_RIG_MICDATA,
    .port_type = RIG_PORT_NONE,
    .has_get_level =  RIG_LEVEL_AF | RIG_LEVEL_RF,
    .has_set_level =  RIG_LEVEL_SET(RIG_LEVEL_AF | RIG_LEVEL_RF),
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

    .set_vfo = zynq7000_set_vfo,
    .get_vfo = zynq7000_get_vfo,
    .set_freq = zynq7000_set_freq,
    .get_freq = zynq7000_get_freq,

    .get_mode = zynq7000_get_mode,
    .set_mode = zynq7000_set_mode,

    .set_ptt = zynq7000_set_ptt,
    .get_ptt = zynq7000_get_ptt,

    .set_level = zynq7000_set_level,
    .get_level = zynq7000_get_level,

    .hamlib_check_rig_caps = HAMLIB_CHECK_RIG_CAPS
};
