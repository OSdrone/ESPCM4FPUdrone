################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
FuncionesRTOS/AHRS_TASK.obj: ../FuncionesRTOS/AHRS_TASK.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-arm_15.12.1.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me -O2 --include_path="C:/ti/ccsv6/tools/compiler/ti-cgt-arm_15.12.1.LTS/include" --include_path="C:/Users/Ruben/Dropbox/TFG/Programacion/CMSIS-SP-00300-r4p3-00rel0/CMSIS/Include" --include_path="C:/ti/tirtos_tivac_2_12_01_33/products/TivaWare_C_Series-2.1.0.12573c" -g --gcc --define=ccs="ccs" --define=ARM_MATH_CM4 --define=__GNUC__ --define=__FPU_PRESENT=1 --define=_LINKAGE --define=_CODE_ACCESS="" --define=_DATA_ACCESS="" --define=PART_TM4C123GH6PM --define=ccs --define=TIVAWARE --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi --ual --preproc_with_compile --preproc_dependency="FuncionesRTOS/AHRS_TASK.pp" --obj_directory="FuncionesRTOS" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

FuncionesRTOS/Control_TASK.obj: ../FuncionesRTOS/Control_TASK.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-arm_15.12.1.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me -O2 --include_path="C:/ti/ccsv6/tools/compiler/ti-cgt-arm_15.12.1.LTS/include" --include_path="C:/Users/Ruben/Dropbox/TFG/Programacion/CMSIS-SP-00300-r4p3-00rel0/CMSIS/Include" --include_path="C:/ti/tirtos_tivac_2_12_01_33/products/TivaWare_C_Series-2.1.0.12573c" -g --gcc --define=ccs="ccs" --define=ARM_MATH_CM4 --define=__GNUC__ --define=__FPU_PRESENT=1 --define=_LINKAGE --define=_CODE_ACCESS="" --define=_DATA_ACCESS="" --define=PART_TM4C123GH6PM --define=ccs --define=TIVAWARE --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi --ual --preproc_with_compile --preproc_dependency="FuncionesRTOS/Control_TASK.pp" --obj_directory="FuncionesRTOS" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

FuncionesRTOS/Coordinador_TASK.obj: ../FuncionesRTOS/Coordinador_TASK.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-arm_15.12.1.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me -O2 --include_path="C:/ti/ccsv6/tools/compiler/ti-cgt-arm_15.12.1.LTS/include" --include_path="C:/Users/Ruben/Dropbox/TFG/Programacion/CMSIS-SP-00300-r4p3-00rel0/CMSIS/Include" --include_path="C:/ti/tirtos_tivac_2_12_01_33/products/TivaWare_C_Series-2.1.0.12573c" -g --gcc --define=ccs="ccs" --define=ARM_MATH_CM4 --define=__GNUC__ --define=__FPU_PRESENT=1 --define=_LINKAGE --define=_CODE_ACCESS="" --define=_DATA_ACCESS="" --define=PART_TM4C123GH6PM --define=ccs --define=TIVAWARE --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi --ual --preproc_with_compile --preproc_dependency="FuncionesRTOS/Coordinador_TASK.pp" --obj_directory="FuncionesRTOS" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

FuncionesRTOS/IMU_TASK.obj: ../FuncionesRTOS/IMU_TASK.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-arm_15.12.1.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me -O2 --include_path="C:/ti/ccsv6/tools/compiler/ti-cgt-arm_15.12.1.LTS/include" --include_path="C:/Users/Ruben/Dropbox/TFG/Programacion/CMSIS-SP-00300-r4p3-00rel0/CMSIS/Include" --include_path="C:/ti/tirtos_tivac_2_12_01_33/products/TivaWare_C_Series-2.1.0.12573c" -g --gcc --define=ccs="ccs" --define=ARM_MATH_CM4 --define=__GNUC__ --define=__FPU_PRESENT=1 --define=_LINKAGE --define=_CODE_ACCESS="" --define=_DATA_ACCESS="" --define=PART_TM4C123GH6PM --define=ccs --define=TIVAWARE --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi --ual --preproc_with_compile --preproc_dependency="FuncionesRTOS/IMU_TASK.pp" --obj_directory="FuncionesRTOS" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


