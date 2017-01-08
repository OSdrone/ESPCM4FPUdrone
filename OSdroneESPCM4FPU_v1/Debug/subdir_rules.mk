################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
AHRS.obj: ../AHRS.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-arm_15.12.1.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me -O2 --include_path="C:/ti/ccsv6/tools/compiler/ti-cgt-arm_15.12.1.LTS/include" --include_path="C:/Users/Ruben/Dropbox/TFG/Programacion/CMSIS-SP-00300-r4p3-00rel0/CMSIS/Include" --include_path="C:/ti/tirtos_tivac_2_12_01_33/products/TivaWare_C_Series-2.1.0.12573c" -g --gcc --define=ccs="ccs" --define=ARM_MATH_CM4 --define=__GNUC__ --define=__FPU_PRESENT=1 --define=_LINKAGE --define=_CODE_ACCESS="" --define=_DATA_ACCESS="" --define=PART_TM4C123GH6PM --define=ccs --define=TIVAWARE --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi --ual --preproc_with_compile --preproc_dependency="AHRS.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

configPkg/linker.cmd: ../Config.cfg
	@echo 'Building file: $<'
	@echo 'Invoking: XDCtools'
	"C:/ti/xdctools_3_31_01_33_core/xs" --xdcpath="C:/ti/tirtos_tivac_2_12_01_33/packages;C:/ti/tirtos_tivac_2_12_01_33/products/bios_6_41_04_54/packages;C:/ti/tirtos_tivac_2_12_01_33/products/ndk_2_24_02_31/packages;C:/ti/tirtos_tivac_2_12_01_33/products/uia_2_00_02_39/packages;C:/ti/ccsv6/ccs_base;" xdc.tools.configuro -o configPkg -t ti.targets.arm.elf.M4F -p ti.platforms.tiva:TM4C123GH6PM -r release -c "C:/ti/ccsv6/tools/compiler/ti-cgt-arm_15.12.1.LTS" --compileOptions "-mv7M4 --code_state=16 --float_support=FPv4SPD16 -me -O2 --include_path=\"C:/ti/ccsv6/tools/compiler/ti-cgt-arm_15.12.1.LTS/include\" --include_path=\"C:/Users/Ruben/Dropbox/TFG/Programacion/CMSIS-SP-00300-r4p3-00rel0/CMSIS/Include\" --include_path=\"C:/ti/tirtos_tivac_2_12_01_33/products/TivaWare_C_Series-2.1.0.12573c\" -g --gcc --define=ccs=\"ccs\" --define=ARM_MATH_CM4 --define=__GNUC__ --define=__FPU_PRESENT=1 --define=_LINKAGE --define=_CODE_ACCESS=\"\" --define=_DATA_ACCESS=\"\" --define=PART_TM4C123GH6PM --define=ccs --define=TIVAWARE --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi --ual  " "$<"
	@echo 'Finished building: $<'
	@echo ' '

configPkg/compiler.opt: | configPkg/linker.cmd
configPkg/: | configPkg/linker.cmd

Funciones_Transferencia.obj: ../Funciones_Transferencia.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-arm_15.12.1.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me -O2 --include_path="C:/ti/ccsv6/tools/compiler/ti-cgt-arm_15.12.1.LTS/include" --include_path="C:/Users/Ruben/Dropbox/TFG/Programacion/CMSIS-SP-00300-r4p3-00rel0/CMSIS/Include" --include_path="C:/ti/tirtos_tivac_2_12_01_33/products/TivaWare_C_Series-2.1.0.12573c" -g --gcc --define=ccs="ccs" --define=ARM_MATH_CM4 --define=__GNUC__ --define=__FPU_PRESENT=1 --define=_LINKAGE --define=_CODE_ACCESS="" --define=_DATA_ACCESS="" --define=PART_TM4C123GH6PM --define=ccs --define=TIVAWARE --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi --ual --preproc_with_compile --preproc_dependency="Funciones_Transferencia.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

Principal.obj: ../Principal.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-arm_15.12.1.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me -O2 --include_path="C:/ti/ccsv6/tools/compiler/ti-cgt-arm_15.12.1.LTS/include" --include_path="C:/Users/Ruben/Dropbox/TFG/Programacion/CMSIS-SP-00300-r4p3-00rel0/CMSIS/Include" --include_path="C:/ti/tirtos_tivac_2_12_01_33/products/TivaWare_C_Series-2.1.0.12573c" -g --gcc --define=ccs="ccs" --define=ARM_MATH_CM4 --define=__GNUC__ --define=__FPU_PRESENT=1 --define=_LINKAGE --define=_CODE_ACCESS="" --define=_DATA_ACCESS="" --define=PART_TM4C123GH6PM --define=ccs --define=TIVAWARE --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi --ual --preproc_with_compile --preproc_dependency="Principal.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

Quad_board.obj: ../Quad_board.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-arm_15.12.1.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me -O2 --include_path="C:/ti/ccsv6/tools/compiler/ti-cgt-arm_15.12.1.LTS/include" --include_path="C:/Users/Ruben/Dropbox/TFG/Programacion/CMSIS-SP-00300-r4p3-00rel0/CMSIS/Include" --include_path="C:/ti/tirtos_tivac_2_12_01_33/products/TivaWare_C_Series-2.1.0.12573c" -g --gcc --define=ccs="ccs" --define=ARM_MATH_CM4 --define=__GNUC__ --define=__FPU_PRESENT=1 --define=_LINKAGE --define=_CODE_ACCESS="" --define=_DATA_ACCESS="" --define=PART_TM4C123GH6PM --define=ccs --define=TIVAWARE --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi --ual --preproc_with_compile --preproc_dependency="Quad_board.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

Servidores.obj: ../Servidores.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-arm_15.12.1.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me -O2 --include_path="C:/ti/ccsv6/tools/compiler/ti-cgt-arm_15.12.1.LTS/include" --include_path="C:/Users/Ruben/Dropbox/TFG/Programacion/CMSIS-SP-00300-r4p3-00rel0/CMSIS/Include" --include_path="C:/ti/tirtos_tivac_2_12_01_33/products/TivaWare_C_Series-2.1.0.12573c" -g --gcc --define=ccs="ccs" --define=ARM_MATH_CM4 --define=__GNUC__ --define=__FPU_PRESENT=1 --define=_LINKAGE --define=_CODE_ACCESS="" --define=_DATA_ACCESS="" --define=PART_TM4C123GH6PM --define=ccs --define=TIVAWARE --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi --ual --preproc_with_compile --preproc_dependency="Servidores.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

Transmisores.obj: ../Transmisores.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-arm_15.12.1.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me -O2 --include_path="C:/ti/ccsv6/tools/compiler/ti-cgt-arm_15.12.1.LTS/include" --include_path="C:/Users/Ruben/Dropbox/TFG/Programacion/CMSIS-SP-00300-r4p3-00rel0/CMSIS/Include" --include_path="C:/ti/tirtos_tivac_2_12_01_33/products/TivaWare_C_Series-2.1.0.12573c" -g --gcc --define=ccs="ccs" --define=ARM_MATH_CM4 --define=__GNUC__ --define=__FPU_PRESENT=1 --define=_LINKAGE --define=_CODE_ACCESS="" --define=_DATA_ACCESS="" --define=PART_TM4C123GH6PM --define=ccs --define=TIVAWARE --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi --ual --preproc_with_compile --preproc_dependency="Transmisores.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


