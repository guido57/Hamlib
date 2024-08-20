# Created by and for Qt Creator This file was created for editing the project sources only.
# You may attempt to use it for building too, by modifying this file here.

#TARGET = Hamlib

QT = core gui widgets

HEADERS = \
   $$PWD/amplifiers/elecraft/kpa.h \
   $$PWD/amplifiers/expert/expert.h \
   $$PWD/amplifiers/gemini/gemini.h \
   $$PWD/android/config.h \
   $$PWD/android/ltdl.h \
   $$PWD/extra/gnuradio/am.h \
   $$PWD/extra/gnuradio/demod.h \
   $$PWD/extra/gnuradio/gnuradio.h \
   $$PWD/extra/gnuradio/gr_priv.h \
   $$PWD/extra/gnuradio/HrAGC.h \
   $$PWD/extra/gnuradio/nfm.h \
   $$PWD/extra/gnuradio/ssb.h \
   $$PWD/extra/gnuradio/wfm.h \
   $$PWD/include/hamlib/ampclass.h \
   $$PWD/include/hamlib/amplifier.h \
   $$PWD/include/hamlib/amplist.h \
   $$PWD/include/hamlib/config.h \
   $$PWD/include/hamlib/multicast.h \
   $$PWD/include/hamlib/rig.h \
   $$PWD/include/hamlib/rig_dll.h \
   $$PWD/include/hamlib/rigclass.h \
   $$PWD/include/hamlib/riglist.h \
   $$PWD/include/hamlib/rotator.h \
   $$PWD/include/hamlib/rotclass.h \
   $$PWD/include/hamlib/rotlist.h \
   $$PWD/include/hamlib/winpthreads.h \
   $$PWD/include/bandplan.h \
   $$PWD/include/num_stdio.h \
   $$PWD/lib/asyncpipe.h \
   $$PWD/lib/cJSON.h \
   $$PWD/lib/getopt.h \
   $$PWD/lib/win32termios.h \
   $$PWD/rigs/adat/adat.h \
   $$PWD/rigs/adat/adt_200a.h \
   $$PWD/rigs/alinco/alinco.h \
   $$PWD/rigs/anytone/anytone.h \
   $$PWD/rigs/aor/aor.h \
   $$PWD/rigs/aor/ar7030p.h \
   $$PWD/rigs/barrett/barrett.h \
   $$PWD/rigs/codan/codan.h \
   $$PWD/rigs/dorji/dorji.h \
   $$PWD/rigs/dorji/dra818.h \
   $$PWD/rigs/drake/drake.h \
   $$PWD/rigs/dummy/amp_dummy.h \
   $$PWD/rigs/dummy/dummy.h \
   $$PWD/rigs/dummy/dummy_common.h \
   $$PWD/rigs/dummy/flrig.h \
   $$PWD/rigs/dummy/rot_dummy.h \
   $$PWD/rigs/dummy/trxmanager.h \
   $$PWD/rigs/dummy/zynq7000.h \
   $$PWD/rigs/dummy/uio.h \
   $$PWD/rigs/dummy/uio_wrapper.h \
   $$PWD/rigs/elad/elad.h \
   $$PWD/rigs/flexradio/flexradio.h \
   $$PWD/rigs/gomspace/gs100.h \
   $$PWD/rigs/icmarine/icm710.h \
   $$PWD/rigs/icmarine/icmarine.h \
   $$PWD/rigs/icom/frame.h \
   $$PWD/rigs/icom/ic7300.h \
   $$PWD/rigs/icom/icom.h \
   $$PWD/rigs/icom/icom_defs.h \
   $$PWD/rigs/icom/level_gran_icom.h \
   $$PWD/rigs/icom/optoscan.h \
   $$PWD/rigs/jrc/jrc.h \
   $$PWD/rigs/kachina/kachina.h \
   $$PWD/rigs/kenwood/elecraft.h \
   $$PWD/rigs/kenwood/flex.h \
   $$PWD/rigs/kenwood/ic10.h \
   $$PWD/rigs/kenwood/kenwood.h \
   $$PWD/rigs/kenwood/level_gran_elecraft.h \
   $$PWD/rigs/kenwood/level_gran_kenwood.h \
   $$PWD/rigs/kenwood/th.h \
   $$PWD/rigs/kenwood/ts990s.h \
   $$PWD/rigs/kit/funcube.h \
   $$PWD/rigs/kit/kit.h \
   $$PWD/rigs/kit/si570avrusb.h \
   $$PWD/rigs/kit/usrp_impl.h \
   $$PWD/rigs/lowe/lowe.h \
   $$PWD/rigs/mds/mds.h \
   $$PWD/rigs/pcr/pcr.h \
   $$PWD/rigs/prm80/prm80.h \
   $$PWD/rigs/racal/ra37xx.h \
   $$PWD/rigs/racal/racal.h \
   $$PWD/rigs/rft/rft.h \
   $$PWD/rigs/rs/ek89x.h \
   $$PWD/rigs/rs/gp2000.h \
   $$PWD/rigs/rs/rs.h \
   $$PWD/rigs/skanti/skanti.h \
   $$PWD/rigs/tapr/tapr.h \
   $$PWD/rigs/tentec/orion.h \
   $$PWD/rigs/tentec/rx331.h \
   $$PWD/rigs/tentec/tentec.h \
   $$PWD/rigs/tentec/tentec2.h \
   $$PWD/rigs/tentec/tt550.h \
   $$PWD/rigs/tuner/tuner.h \
   $$PWD/rigs/tuner/videodev.h \
   $$PWD/rigs/tuner/videodev2.h \
   $$PWD/rigs/uniden/uniden.h \
   $$PWD/rigs/uniden/uniden_digital.h \
   $$PWD/rigs/winradio/linradio/radio_ioctl.h \
   $$PWD/rigs/winradio/linradio/wrapi.h \
   $$PWD/rigs/winradio/linradio/wrg313api.h \
   $$PWD/rigs/winradio/winradio.h \
   $$PWD/rigs/wj/wj.h \
   $$PWD/rigs/yaesu/frg100.h \
   $$PWD/rigs/yaesu/ft100.h \
   $$PWD/rigs/yaesu/ft1000d.h \
   $$PWD/rigs/yaesu/ft1000mp.h \
   $$PWD/rigs/yaesu/ft1200.h \
   $$PWD/rigs/yaesu/ft2000.h \
   $$PWD/rigs/yaesu/ft450.h \
   $$PWD/rigs/yaesu/ft5000.h \
   $$PWD/rigs/yaesu/ft600.h \
   $$PWD/rigs/yaesu/ft747.h \
   $$PWD/rigs/yaesu/ft757gx.h \
   $$PWD/rigs/yaesu/ft767gx.h \
   $$PWD/rigs/yaesu/ft817.h \
   $$PWD/rigs/yaesu/ft840.h \
   $$PWD/rigs/yaesu/ft847.h \
   $$PWD/rigs/yaesu/ft857.h \
   $$PWD/rigs/yaesu/ft890.h \
   $$PWD/rigs/yaesu/ft891.h \
   $$PWD/rigs/yaesu/ft897.h \
   $$PWD/rigs/yaesu/ft900.h \
   $$PWD/rigs/yaesu/ft9000.h \
   $$PWD/rigs/yaesu/ft920.h \
   $$PWD/rigs/yaesu/ft950.h \
   $$PWD/rigs/yaesu/ft980.h \
   $$PWD/rigs/yaesu/ft990.h \
   $$PWD/rigs/yaesu/ft990v12.h \
   $$PWD/rigs/yaesu/ft991.h \
   $$PWD/rigs/yaesu/ftdx10.h \
   $$PWD/rigs/yaesu/ftdx101.h \
   $$PWD/rigs/yaesu/level_gran_yaesu.h \
   $$PWD/rigs/yaesu/newcat.h \
   $$PWD/rigs/yaesu/vx1700.h \
   $$PWD/rigs/yaesu/yaesu.h \
   $$PWD/rotators/androidsensor/androidsensor.h \
#   $$PWD/rotators/androidsensor/ndkimu.h \
   $$PWD/rotators/apex/apex.h \
   $$PWD/rotators/ars/ars.h \
   $$PWD/rotators/celestron/celestron.h \
   $$PWD/rotators/easycomm/easycomm.h \
   $$PWD/rotators/ether6/ether6.h \
   $$PWD/rotators/flir/flir.h \
   $$PWD/rotators/fodtrack/fodtrack.h \
   $$PWD/rotators/gs232a/gs232a.h \
   $$PWD/rotators/heathkit/hd1780.h \
   $$PWD/rotators/indi/indi_wrapper.h \
#   $$PWD/rotators/indi/indi_wrapper.hpp \
   $$PWD/rotators/ioptron/rot_ioptron.h \
   $$PWD/rotators/m2/rc2800.h \
   $$PWD/rotators/meade/meade.h \
   $$PWD/rotators/prosistel/prosistel.h \
   $$PWD/rotators/radant/radant.h \
   $$PWD/rotators/rotorez/rotorez.h \
   $$PWD/rotators/saebrtrack/saebrtrack.h \
   $$PWD/rotators/sartek/sartek.h \
   $$PWD/rotators/satel/satel.h \
   $$PWD/rotators/spid/spid.h \
   $$PWD/rotators/ts7400/include/ep93xx_adc.h \
   $$PWD/rotators/ts7400/include/io.h \
   $$PWD/rotators/ts7400/include/peekpoke.h \
   $$PWD/rotators/ts7400/include/readADC.h \
   $$PWD/rotators/ts7400/ts7400.h \
   $$PWD/scripts/MSVC/2022/x64/hamlibTest/framework.h \
   $$PWD/scripts/MSVC/2022/x64/hamlibTest/hamlibTest.h \
   $$PWD/scripts/MSVC/2022/x64/hamlibTest/Resource.h \
   $$PWD/scripts/MSVC/2022/x64/hamlibTest/targetver.h \
   $$PWD/scripts/MSVC/2022/x86/hamlibTest/framework.h \
   $$PWD/scripts/MSVC/2022/x86/hamlibTest/hamlibTest.h \
   $$PWD/scripts/MSVC/2022/x86/hamlibTest/Resource.h \
   $$PWD/scripts/MSVC/2022/x86/hamlibTest/targetver.h \
   $$PWD/security/aes.h \
   $$PWD/security/AESStringCrypt.h \
   $$PWD/security/md5.h \
   $$PWD/security/password.h \
   $$PWD/security/security.h \
   $$PWD/security/sha256.h \
   $$PWD/src/amp_conf.h \
   $$PWD/src/cache.h \
   $$PWD/src/cal.h \
   $$PWD/src/cm108.h \
   $$PWD/src/event.h \
   $$PWD/src/gpio.h \
   $$PWD/src/hamlibdatetime.h \
   $$PWD/src/idx_builtin.h \
   $$PWD/src/iofunc.h \
   $$PWD/src/microham.h \
   $$PWD/src/misc.h \
   $$PWD/src/network.h \
   $$PWD/src/par_nt.h \
   $$PWD/src/parallel.h \
   $$PWD/src/register.h \
   $$PWD/src/rot_conf.h \
   $$PWD/src/serial.h \
   $$PWD/src/sleep.h \
   $$PWD/src/snapshot_data.h \
   $$PWD/src/sprintflst.h \
   $$PWD/src/token.h \
   $$PWD/src/tones.h \
   $$PWD/src/usb_port.h \
   $$PWD/tests/ampctl_parse.h \
   $$PWD/tests/rigctl_parse.h \
   $$PWD/tests/rotctl_parse.h \
   $$PWD/tests/uthash.h \
    rigs/dummy/uio_c.h

SOURCES = \
   $$PWD/amplifiers/elecraft/kpa.c \
   $$PWD/amplifiers/elecraft/kpa1500.c \
   $$PWD/amplifiers/expert/expert.c \
   $$PWD/amplifiers/gemini/dx1200.c \
   $$PWD/amplifiers/gemini/gemini.c \
   $$PWD/android/ltdl.c \
   $$PWD/c++/ampclass.cc \
   $$PWD/c++/rigclass.cc \
   $$PWD/c++/rotclass.cc \
   $$PWD/c++/testcpp.cc \
#   $$PWD/extra/gnuradio/gnuradio.cc \
   $$PWD/extra/gnuradio/gr.c \
   $$PWD/extra/gnuradio/graudio.c \
   $$PWD/extra/gnuradio/mc4020.c \
#   $$PWD/extra/gnuradio/testgr.cc \
   $$PWD/lib/asyncpipe.c \
   $$PWD/lib/cJSON.c \
   $$PWD/lib/dummy.c \
   $$PWD/lib/getaddrinfo.c \
   $$PWD/lib/getopt.c \
   $$PWD/lib/getopt_long.c \
   $$PWD/lib/gettimeofday.c \
   $$PWD/lib/precise_time.c \
   $$PWD/lib/termios.c \
   $$PWD/lib/usleep.c \
   $$PWD/rigs/adat/adat.c \
   $$PWD/rigs/adat/adt_200a.c \
   $$PWD/rigs/alinco/alinco.c \
   $$PWD/rigs/alinco/dx77.c \
   $$PWD/rigs/alinco/dxsr8.c \
   $$PWD/rigs/anytone/anytone.c \
   $$PWD/rigs/anytone/d578.c \
   $$PWD/rigs/aor/aor.c \
   $$PWD/rigs/aor/ar2700.c \
   $$PWD/rigs/aor/ar3000.c \
   $$PWD/rigs/aor/ar3030.c \
   $$PWD/rigs/aor/ar5000.c \
   $$PWD/rigs/aor/ar7030.c \
   $$PWD/rigs/aor/ar7030p.c \
   $$PWD/rigs/aor/ar7030p_utils.c \
   $$PWD/rigs/aor/ar8000.c \
   $$PWD/rigs/aor/ar8200.c \
   $$PWD/rigs/aor/ar8600.c \
   $$PWD/rigs/aor/sr2200.c \
   $$PWD/rigs/barrett/4050.c \
   $$PWD/rigs/barrett/950.c \
   $$PWD/rigs/barrett/barrett.c \
   $$PWD/rigs/codan/codan.c \
   $$PWD/rigs/dorji/dorji.c \
   $$PWD/rigs/dorji/dra818.c \
   $$PWD/rigs/drake/drake.c \
   $$PWD/rigs/drake/r8a.c \
   $$PWD/rigs/drake/r8b.c \
   $$PWD/rigs/dummy/aclog.c \
   $$PWD/rigs/dummy/amp_dummy.c \
   $$PWD/rigs/dummy/dummy.c \
   $$PWD/rigs/dummy/dummy_common.c \
   $$PWD/rigs/dummy/flrig.c \
   $$PWD/rigs/dummy/netampctl.c \
   $$PWD/rigs/dummy/netrigctl.c \
   $$PWD/rigs/dummy/netrotctl.c \
   $$PWD/rigs/dummy/rot_dummy.c \
   $$PWD/rigs/dummy/sdrsharp.c \
   $$PWD/rigs/dummy/zynq7000.c \
   $$PWD/rigs/dummy/uio.cpp \
   $$PWD/rigs/dummy/uio_wrapper.cpp \
   $$PWD/rigs/dummy/tci1x.c \
   $$PWD/rigs/dummy/trxmanager.c \
   $$PWD/rigs/elad/elad.c \
   $$PWD/rigs/elad/fdm_duo.c \
   $$PWD/rigs/flexradio/dttsp.c \
   $$PWD/rigs/flexradio/flexradio.c \
   $$PWD/rigs/flexradio/sdr1k.c \
   $$PWD/rigs/gomspace/gs100.c \
   $$PWD/rigs/icmarine/icm700pro.c \
   $$PWD/rigs/icmarine/icm710.c \
   $$PWD/rigs/icmarine/icm802.c \
   $$PWD/rigs/icmarine/icm803.c \
   $$PWD/rigs/icmarine/icmarine.c \
   $$PWD/rigs/icom/delta2.c \
   $$PWD/rigs/icom/frame.c \
   $$PWD/rigs/icom/ic1275.c \
   $$PWD/rigs/icom/ic271.c \
   $$PWD/rigs/icom/ic2730.c \
   $$PWD/rigs/icom/ic275.c \
   $$PWD/rigs/icom/ic471.c \
   $$PWD/rigs/icom/ic475.c \
   $$PWD/rigs/icom/ic7000.c \
   $$PWD/rigs/icom/ic703.c \
   $$PWD/rigs/icom/ic706.c \
   $$PWD/rigs/icom/ic707.c \
   $$PWD/rigs/icom/ic7100.c \
   $$PWD/rigs/icom/ic718.c \
   $$PWD/rigs/icom/ic7200.c \
   $$PWD/rigs/icom/ic725.c \
   $$PWD/rigs/icom/ic726.c \
   $$PWD/rigs/icom/ic728.c \
   $$PWD/rigs/icom/ic7300.c \
   $$PWD/rigs/icom/ic735.c \
   $$PWD/rigs/icom/ic736.c \
   $$PWD/rigs/icom/ic737.c \
   $$PWD/rigs/icom/ic738.c \
   $$PWD/rigs/icom/ic7410.c \
   $$PWD/rigs/icom/ic746.c \
   $$PWD/rigs/icom/ic751.c \
   $$PWD/rigs/icom/ic756.c \
   $$PWD/rigs/icom/ic7600.c \
   $$PWD/rigs/icom/ic761.c \
   $$PWD/rigs/icom/ic7610.c \
   $$PWD/rigs/icom/ic765.c \
   $$PWD/rigs/icom/ic7700.c \
   $$PWD/rigs/icom/ic775.c \
   $$PWD/rigs/icom/ic78.c \
   $$PWD/rigs/icom/ic7800.c \
   $$PWD/rigs/icom/ic781.c \
   $$PWD/rigs/icom/ic785x.c \
   $$PWD/rigs/icom/ic820h.c \
   $$PWD/rigs/icom/ic821h.c \
   $$PWD/rigs/icom/ic910.c \
   $$PWD/rigs/icom/ic9100.c \
   $$PWD/rigs/icom/ic92d.c \
   $$PWD/rigs/icom/ic970.c \
   $$PWD/rigs/icom/icf8101.c \
   $$PWD/rigs/icom/icom.c \
   $$PWD/rigs/icom/icr10.c \
   $$PWD/rigs/icom/icr20.c \
   $$PWD/rigs/icom/icr30.c \
   $$PWD/rigs/icom/icr6.c \
   $$PWD/rigs/icom/icr7000.c \
   $$PWD/rigs/icom/icr71.c \
   $$PWD/rigs/icom/icr72.c \
   $$PWD/rigs/icom/icr75.c \
   $$PWD/rigs/icom/icr8500.c \
   $$PWD/rigs/icom/icr8600.c \
   $$PWD/rigs/icom/icr9000.c \
   $$PWD/rigs/icom/icr9500.c \
   $$PWD/rigs/icom/icrx7.c \
   $$PWD/rigs/icom/id1.c \
   $$PWD/rigs/icom/id31.c \
   $$PWD/rigs/icom/id4100.c \
   $$PWD/rigs/icom/id51.c \
   $$PWD/rigs/icom/id5100.c \
   $$PWD/rigs/icom/omni.c \
   $$PWD/rigs/icom/optoscan.c \
   $$PWD/rigs/icom/os456.c \
   $$PWD/rigs/icom/os535.c \
   $$PWD/rigs/icom/perseus.c \
   $$PWD/rigs/icom/xiegu.c \
   $$PWD/rigs/jrc/jrc.c \
   $$PWD/rigs/jrc/jst145.c \
   $$PWD/rigs/jrc/nrd525.c \
   $$PWD/rigs/jrc/nrd535.c \
   $$PWD/rigs/jrc/nrd545.c \
   $$PWD/rigs/kachina/505dsp.c \
   $$PWD/rigs/kachina/kachina.c \
   $$PWD/rigs/kenwood/elecraft.c \
   $$PWD/rigs/kenwood/flex.c \
   $$PWD/rigs/kenwood/flex6xxx.c \
   $$PWD/rigs/kenwood/ic10.c \
   $$PWD/rigs/kenwood/k2.c \
   $$PWD/rigs/kenwood/k3.c \
   $$PWD/rigs/kenwood/kenwood.c \
   $$PWD/rigs/kenwood/pihpsdr.c \
   $$PWD/rigs/kenwood/r5000.c \
   $$PWD/rigs/kenwood/th.c \
   $$PWD/rigs/kenwood/thd7.c \
   $$PWD/rigs/kenwood/thd72.c \
   $$PWD/rigs/kenwood/thd74.c \
   $$PWD/rigs/kenwood/thf6a.c \
   $$PWD/rigs/kenwood/thf7.c \
   $$PWD/rigs/kenwood/thg71.c \
   $$PWD/rigs/kenwood/tmd700.c \
   $$PWD/rigs/kenwood/tmd710.c \
   $$PWD/rigs/kenwood/tmv7.c \
   $$PWD/rigs/kenwood/transfox.c \
   $$PWD/rigs/kenwood/trc80.c \
   $$PWD/rigs/kenwood/ts140.c \
   $$PWD/rigs/kenwood/ts2000.c \
   $$PWD/rigs/kenwood/ts440.c \
   $$PWD/rigs/kenwood/ts450s.c \
   $$PWD/rigs/kenwood/ts480.c \
   $$PWD/rigs/kenwood/ts50s.c \
   $$PWD/rigs/kenwood/ts570.c \
   $$PWD/rigs/kenwood/ts590.c \
   $$PWD/rigs/kenwood/ts680.c \
   $$PWD/rigs/kenwood/ts690.c \
   $$PWD/rigs/kenwood/ts711.c \
   $$PWD/rigs/kenwood/ts790.c \
   $$PWD/rigs/kenwood/ts811.c \
   $$PWD/rigs/kenwood/ts850.c \
   $$PWD/rigs/kenwood/ts870s.c \
   $$PWD/rigs/kenwood/ts890s.c \
   $$PWD/rigs/kenwood/ts930.c \
   $$PWD/rigs/kenwood/ts940.c \
   $$PWD/rigs/kenwood/ts950.c \
   $$PWD/rigs/kenwood/ts990s.c \
   $$PWD/rigs/kenwood/tx500.c \
   $$PWD/rigs/kenwood/xg3.c \
   $$PWD/rigs/kit/dds60.c \
   $$PWD/rigs/kit/drt1.c \
   $$PWD/rigs/kit/dwt.c \
   $$PWD/rigs/kit/elektor304.c \
   $$PWD/rigs/kit/elektor507.c \
   $$PWD/rigs/kit/fifisdr.c \
   $$PWD/rigs/kit/funcube.c \
   $$PWD/rigs/kit/hiqsdr.c \
   $$PWD/rigs/kit/kit.c \
   $$PWD/rigs/kit/miniVNA.c \
   $$PWD/rigs/kit/pcrotor.c \
   $$PWD/rigs/kit/rs_hfiq.c \
   $$PWD/rigs/kit/si570avrusb.c \
   $$PWD/rigs/kit/usrp.c \
   $$PWD/rigs/kit/usrp_impl.cc \
   $$PWD/rigs/lowe/hf235.c \
   $$PWD/rigs/lowe/lowe.c \
   $$PWD/rigs/mds/4710.c \
   $$PWD/rigs/mds/9710.c \
   $$PWD/rigs/mds/mds.c \
   $$PWD/rigs/mds/serialnum.c \
   $$PWD/rigs/pcr/pcr.c \
   $$PWD/rigs/pcr/pcr100.c \
   $$PWD/rigs/pcr/pcr1000.c \
   $$PWD/rigs/pcr/pcr1500.c \
   $$PWD/rigs/pcr/pcr2500.c \
   $$PWD/rigs/prm80/prm80.c \
   $$PWD/rigs/prm80/prm8060.c \
   $$PWD/rigs/racal/ra3702.c \
   $$PWD/rigs/racal/ra37xx.c \
   $$PWD/rigs/racal/ra6790.c \
   $$PWD/rigs/racal/racal.c \
   $$PWD/rigs/rft/ekd500.c \
   $$PWD/rigs/rft/rft.c \
   $$PWD/rigs/rs/eb200.c \
   $$PWD/rigs/rs/ek89x.c \
   $$PWD/rigs/rs/esmc.c \
   $$PWD/rigs/rs/gp2000.c \
   $$PWD/rigs/rs/rs.c \
   $$PWD/rigs/rs/xk2100.c \
   $$PWD/rigs/skanti/skanti.c \
   $$PWD/rigs/skanti/trp8000.c \
   $$PWD/rigs/skanti/trp8255.c \
   $$PWD/rigs/tapr/dsp10.c \
   $$PWD/rigs/tapr/tapr.c \
   $$PWD/rigs/tentec/argonaut.c \
   $$PWD/rigs/tentec/jupiter.c \
   $$PWD/rigs/tentec/omnivii.c \
   $$PWD/rigs/tentec/orion.c \
   $$PWD/rigs/tentec/paragon.c \
   $$PWD/rigs/tentec/pegasus.c \
   $$PWD/rigs/tentec/rx320.c \
   $$PWD/rigs/tentec/rx331.c \
   $$PWD/rigs/tentec/rx340.c \
   $$PWD/rigs/tentec/rx350.c \
   $$PWD/rigs/tentec/tentec.c \
   $$PWD/rigs/tentec/tentec2.c \
   $$PWD/rigs/tentec/tt550.c \
   $$PWD/rigs/tuner/tuner.c \
   $$PWD/rigs/tuner/v4l.c \
   $$PWD/rigs/tuner/v4l2.c \
   $$PWD/rigs/uniden/bc245.c \
   $$PWD/rigs/uniden/bc250.c \
   $$PWD/rigs/uniden/bc780.c \
   $$PWD/rigs/uniden/bc895.c \
   $$PWD/rigs/uniden/bc898.c \
   $$PWD/rigs/uniden/bcd396t.c \
   $$PWD/rigs/uniden/bcd996t.c \
   $$PWD/rigs/uniden/pro2052.c \
   $$PWD/rigs/uniden/uniden.c \
   $$PWD/rigs/uniden/uniden_digital.c \
   $$PWD/rigs/winradio/linradio/wrg313api.c \
   $$PWD/rigs/winradio/g303.c \
   $$PWD/rigs/winradio/g305.c \
   $$PWD/rigs/winradio/g313-posix.c \
#   $$PWD/rigs/winradio/g313-win.c \
   $$PWD/rigs/winradio/winradio.c \
   $$PWD/rigs/winradio/wr1000.c \
   $$PWD/rigs/winradio/wr1500.c \
   $$PWD/rigs/winradio/wr1550.c \
   $$PWD/rigs/winradio/wr3100.c \
   $$PWD/rigs/winradio/wr3150.c \
   $$PWD/rigs/winradio/wr3500.c \
   $$PWD/rigs/winradio/wr3700.c \
   $$PWD/rigs/wj/wj.c \
   $$PWD/rigs/wj/wj8888.c \
   $$PWD/rigs/yaesu/frg100.c \
   $$PWD/rigs/yaesu/frg8800.c \
   $$PWD/rigs/yaesu/frg9600.c \
   $$PWD/rigs/yaesu/ft100.c \
   $$PWD/rigs/yaesu/ft1000d.c \
   $$PWD/rigs/yaesu/ft1000mp.c \
   $$PWD/rigs/yaesu/ft1200.c \
   $$PWD/rigs/yaesu/ft2000.c \
   $$PWD/rigs/yaesu/ft3000.c \
   $$PWD/rigs/yaesu/ft450.c \
   $$PWD/rigs/yaesu/ft5000.c \
   $$PWD/rigs/yaesu/ft600.c \
   $$PWD/rigs/yaesu/ft710.c \
   $$PWD/rigs/yaesu/ft736.c \
   $$PWD/rigs/yaesu/ft747.c \
   $$PWD/rigs/yaesu/ft757gx.c \
   $$PWD/rigs/yaesu/ft767gx.c \
   $$PWD/rigs/yaesu/ft817.c \
   $$PWD/rigs/yaesu/ft840.c \
   $$PWD/rigs/yaesu/ft847.c \
   $$PWD/rigs/yaesu/ft857.c \
   $$PWD/rigs/yaesu/ft890.c \
   $$PWD/rigs/yaesu/ft891.c \
   $$PWD/rigs/yaesu/ft897.c \
   $$PWD/rigs/yaesu/ft900.c \
   $$PWD/rigs/yaesu/ft9000.c \
   $$PWD/rigs/yaesu/ft920.c \
   $$PWD/rigs/yaesu/ft950.c \
   $$PWD/rigs/yaesu/ft980.c \
   $$PWD/rigs/yaesu/ft990.c \
   $$PWD/rigs/yaesu/ft990v12.c \
   $$PWD/rigs/yaesu/ft991.c \
   $$PWD/rigs/yaesu/ftdx10.c \
   $$PWD/rigs/yaesu/ftdx101.c \
   $$PWD/rigs/yaesu/ftdx101mp.c \
   $$PWD/rigs/yaesu/newcat.c \
   $$PWD/rigs/yaesu/vr5000.c \
   $$PWD/rigs/yaesu/vx1700.c \
   $$PWD/rigs/yaesu/yaesu.c \
   $$PWD/rotators/amsat/if100.c \
#   $$PWD/rotators/androidsensor/androidsensor.cpp \
#   $$PWD/rotators/androidsensor/ndkimu.cpp \
   $$PWD/rotators/apex/apex.c \
   $$PWD/rotators/apex/sharedloop.c \
   $$PWD/rotators/ars/ars.c \
   $$PWD/rotators/celestron/celestron.c \
   $$PWD/rotators/cnctrk/cnctrk.c \
   $$PWD/rotators/easycomm/easycomm.c \
   $$PWD/rotators/ether6/ether6.c \
   $$PWD/rotators/flir/flir.c \
   $$PWD/rotators/fodtrack/fodtrack.c \
   $$PWD/rotators/grbltrk/grbltrk.c \
   $$PWD/rotators/gs232a/gs232.c \
   $$PWD/rotators/gs232a/gs232a.c \
   $$PWD/rotators/gs232a/gs232b.c \
   $$PWD/rotators/heathkit/hd1780.c \
   $$PWD/rotators/indi/indi.c \
#   $$PWD/rotators/indi/indi_wrapper.cpp \
   $$PWD/rotators/ioptron/rot_ioptron.c \
   $$PWD/rotators/m2/rc2800.c \
   $$PWD/rotators/meade/meade.c \
   $$PWD/rotators/prosistel/prosistel.c \
   $$PWD/rotators/radant/radant.c \
   $$PWD/rotators/rotorez/rotorez.c \
   $$PWD/rotators/saebrtrack/saebrtrack.c \
   $$PWD/rotators/sartek/sartek.c \
   $$PWD/rotators/satel/satel.c \
   $$PWD/rotators/spid/spid.c \
   $$PWD/rotators/ts7400/include/io.c \
   $$PWD/rotators/ts7400/include/peekpoke.c \
   $$PWD/rotators/ts7400/include/readADC.c \
   $$PWD/rotators/ts7400/include/test7400ADC.c \
   $$PWD/rotators/ts7400/ts7400.c \
#   $$PWD/scripts/MSVC/2022/x64/hamlibTest/hamlibTest.cpp \
#   $$PWD/scripts/MSVC/2022/x64/hamlibTest/hamlibTest2.cpp \
#   $$PWD/scripts/MSVC/2022/x86/hamlibTest/hamlibTest.cpp \
#   $$PWD/scripts/MSVC/2022/x86/hamlibTest/hamlibTest2.cpp \
   $$PWD/security/aes.c \
   $$PWD/security/AESStringCrypt.c \
   $$PWD/security/md5.c \
   $$PWD/security/password.c \
   $$PWD/security/sctest.c \
   $$PWD/security/security.c \
   $$PWD/security/sha256.c \
   $$PWD/simulators/simatd578.c \
   $$PWD/simulators/simelecraft.c \
   $$PWD/simulators/simft736.c \
   $$PWD/simulators/simft747gx.c \
   $$PWD/simulators/simft817.c \
   $$PWD/simulators/simft818.c \
   $$PWD/simulators/simft847.c \
   $$PWD/simulators/simft897.c \
   $$PWD/simulators/simft990.c \
   $$PWD/simulators/simft991.c \
   $$PWD/simulators/simftdx1200.c \
   $$PWD/simulators/simftdx3000.c \
   $$PWD/simulators/simftdx5000.c \
   $$PWD/simulators/simicom.c \
   $$PWD/simulators/simicom7100.c \
   $$PWD/simulators/simicom7300.c \
   $$PWD/simulators/simicom7600.c \
   $$PWD/simulators/simicom905.c \
   $$PWD/simulators/simicom9100.c \
   $$PWD/simulators/simicom9700.c \
   $$PWD/simulators/simid5100.c \
   $$PWD/simulators/simjupiter.c \
   $$PWD/simulators/simkenwood.c \
   $$PWD/simulators/simpowersdr.c \
   $$PWD/simulators/simrotorez.c \
   $$PWD/simulators/simspid.c \
   $$PWD/simulators/simtmd700.c \
   $$PWD/simulators/simts450.c \
   $$PWD/simulators/simts590.c \
   $$PWD/simulators/simyaesu.c \
   $$PWD/src/amp_conf.c \
   $$PWD/src/amp_reg.c \
   $$PWD/src/amp_settings.c \
   $$PWD/src/amplifier.c \
   $$PWD/src/cache.c \
   $$PWD/src/cal.c \
   $$PWD/src/cm108.c \
   $$PWD/src/conf.c \
   $$PWD/src/debug.c \
   $$PWD/src/event.c \
   $$PWD/src/ext.c \
   $$PWD/src/extamp.c \
   $$PWD/src/gpio.c \
   $$PWD/src/iofunc.c \
   $$PWD/src/locator.c \
   $$PWD/src/mem.c \
   $$PWD/src/microham.c \
   $$PWD/src/misc.c \
   $$PWD/src/multicast.c \
   $$PWD/src/network.c \
   $$PWD/src/neverused.c \
   $$PWD/src/parallel.c \
   $$PWD/src/register.c \
   $$PWD/src/rig.c \
   $$PWD/src/rot_conf.c \
   $$PWD/src/rot_ext.c \
   $$PWD/src/rot_reg.c \
   $$PWD/src/rot_settings.c \
   $$PWD/src/rotator.c \
   $$PWD/src/serial.c \
   $$PWD/src/settings.c \
   $$PWD/src/sleep.c \
   $$PWD/src/snapshot_data.c \
   $$PWD/src/sprintflst.c \
   $$PWD/src/tones.c \
   $$PWD/src/usb_port.c \
   $$PWD/tests/ampctl.c \
   $$PWD/tests/ampctl_parse.c \
   $$PWD/tests/ampctld.c \
   $$PWD/tests/cachetest.c \
   $$PWD/tests/cachetest2.c \
   $$PWD/tests/dumpcaps.c \
   $$PWD/tests/dumpcaps_amp.c \
   $$PWD/tests/dumpcaps_rot.c \
   $$PWD/tests/dumpmem.c \
   $$PWD/tests/dumpstate.c \
   $$PWD/tests/example.c \
   $$PWD/tests/func_chk.c \
   $$PWD/tests/hamlibmodels.c \
   $$PWD/tests/listrigs.c \
   $$PWD/tests/memcsv.c \
   $$PWD/tests/memload.c \
   $$PWD/tests/memsave.c \
   $$PWD/tests/rig_bench.c \
   $$PWD/tests/rigctl.c \
   $$PWD/tests/rigctl_parse.c \
   $$PWD/tests/rigctlcom.c \
   $$PWD/tests/rigctld.c \
   $$PWD/tests/rigctlsync.c \
   $$PWD/tests/rigctltcp.c \
#   $$PWD/tests/rigmatrix.c \
   $$PWD/tests/rigmem.c \
   $$PWD/tests/rigsmtr.c \
   $$PWD/tests/rigswr.c \
   $$PWD/tests/rigtestlibusb.c \
   $$PWD/tests/rigtestmcast.c \
   $$PWD/tests/rigtestmcastrx.c \
   $$PWD/tests/rotctl.c \
   $$PWD/tests/rotctl_parse.c \
   $$PWD/tests/rotctld.c \
   $$PWD/tests/sendraw.c \
   $$PWD/tests/simple.c \
   $$PWD/tests/testbcd.c \
   $$PWD/tests/testcache.c \
   $$PWD/tests/testcookie.c \
   $$PWD/tests/testfreq.c \
   $$PWD/tests/testgrid.c \
   $$PWD/tests/testloc.c \
   $$PWD/tests/testnet.c \
   $$PWD/tests/testrig.c \
   $$PWD/tests/testrigcaps.c \
   $$PWD/tests/testrigopen.c \
   $$PWD/tests/testsecurity.c \
   $$PWD/tests/testtrn.c \
    rigs/dummy/uio.c

INCLUDEPATH = \
    $$PWD/amplifiers/elecraft \
    $$PWD/amplifiers/expert \
    $$PWD/amplifiers/gemini \
    $$PWD/android \
    $$PWD/extra/gnuradio \
    $$PWD/include \
    $$PWD/include/hamlib \
    $$PWD/lib \
    $$PWD/rigs/adat \
    $$PWD/rigs/alinco \
    $$PWD/rigs/anytone \
    $$PWD/rigs/aor \
    $$PWD/rigs/barrett \
    $$PWD/rigs/codan \
    $$PWD/rigs/dorji \
    $$PWD/rigs/drake \
    $$PWD/rigs/dummy \
    $$PWD/rigs/elad \
    $$PWD/rigs/flexradio \
    $$PWD/rigs/gomspace \
    $$PWD/rigs/icmarine \
    $$PWD/rigs/icom \
    $$PWD/rigs/jrc \
    $$PWD/rigs/kachina \
    $$PWD/rigs/kenwood \
    $$PWD/rigs/kit \
    $$PWD/rigs/lowe \
    $$PWD/rigs/mds \
    $$PWD/rigs/pcr \
    $$PWD/rigs/prm80 \
    $$PWD/rigs/racal \
    $$PWD/rigs/rft \
    $$PWD/rigs/rs \
    $$PWD/rigs/skanti \
    $$PWD/rigs/tapr \
    $$PWD/rigs/tentec \
    $$PWD/rigs/tuner \
    $$PWD/rigs/uniden \
    $$PWD/rigs/winradio \
    $$PWD/rigs/winradio/linradio \
    $$PWD/rigs/wj \
    $$PWD/rigs/yaesu \
    $$PWD/rotators/androidsensor \
    $$PWD/rotators/apex \
    $$PWD/rotators/ars \
    $$PWD/rotators/celestron \
    $$PWD/rotators/easycomm \
    $$PWD/rotators/ether6 \
    $$PWD/rotators/flir \
    $$PWD/rotators/fodtrack \
    $$PWD/rotators/gs232a \
    $$PWD/rotators/heathkit \
    $$PWD/rotators/indi \
    $$PWD/rotators/ioptron \
    $$PWD/rotators/m2 \
    $$PWD/rotators/meade \
    $$PWD/rotators/prosistel \
    $$PWD/rotators/radant \
    $$PWD/rotators/rotorez \
    $$PWD/rotators/saebrtrack \
    $$PWD/rotators/sartek \
    $$PWD/rotators/satel \
    $$PWD/rotators/spid \
    $$PWD/rotators/ts7400 \
    $$PWD/rotators/ts7400/include \
    $$PWD/scripts/MSVC/2022/x64/hamlibTest \
    $$PWD/scripts/MSVC/2022/x86/hamlibTest \
    $$PWD/security \
    $$PWD/src \
    $$PWD/tests

#DEFINES = 

