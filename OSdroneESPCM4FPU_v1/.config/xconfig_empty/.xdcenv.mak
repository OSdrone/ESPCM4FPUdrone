#
_XDCBUILDCOUNT = 
ifneq (,$(findstring path,$(_USEXDCENV_)))
override XDCPATH = C:/ti/tirtos_tivac_2_12_01_33/packages;C:/ti/tirtos_tivac_2_12_01_33/products/bios_6_41_04_54/packages;C:/ti/tirtos_tivac_2_12_01_33/products/ndk_2_24_02_31/packages;C:/ti/tirtos_tivac_2_12_01_33/products/uia_2_00_02_39/packages;C:/ti/ccsv6/ccs_base;C:/Users/Ruben_User/workspace_v6_1/QuadRotor_V1.3/.config
override XDCROOT = C:/ti/xdctools_3_31_01_33_core
override XDCBUILDCFG = ./config.bld
endif
ifneq (,$(findstring args,$(_USEXDCENV_)))
override XDCARGS = 
override XDCTARGETS = 
endif
#
ifeq (0,1)
PKGPATH = C:/ti/tirtos_tivac_2_12_01_33/packages;C:/ti/tirtos_tivac_2_12_01_33/products/bios_6_41_04_54/packages;C:/ti/tirtos_tivac_2_12_01_33/products/ndk_2_24_02_31/packages;C:/ti/tirtos_tivac_2_12_01_33/products/uia_2_00_02_39/packages;C:/ti/ccsv6/ccs_base;C:/Users/Ruben_User/workspace_v6_1/QuadRotor_V1.3/.config;C:/ti/xdctools_3_31_01_33_core/packages;..
HOSTOS = Windows
endif
