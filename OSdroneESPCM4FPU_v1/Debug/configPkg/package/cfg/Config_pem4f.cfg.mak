# invoke SourceDir generated makefile for Config.pem4f
Config.pem4f: .libraries,Config.pem4f
.libraries,Config.pem4f: package/cfg/Config_pem4f.xdl
	$(MAKE) -f C:\Users\Ruben\workspace_v6_1\OSdroneESPCM4FPU_v1/src/makefile.libs

clean::
	$(MAKE) -f C:\Users\Ruben\workspace_v6_1\OSdroneESPCM4FPU_v1/src/makefile.libs clean

