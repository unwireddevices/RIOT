SHELL := /bin/bash # Use bash syntax

DIRS += $(dir $(wildcard $(addsuffix /Makefile, ${USEMODULE})))
INCLUDES += $(addprefix -I../../unwired-modules/, $(wildcard $(addsuffix /include, $(filter umdk-%, ${USEMODULE}))))
CFLAGS += $(addprefix -D, $(subst umdk-, umdk_, $(filter umdk-%, ${USEMODULE})))
UMDK-INCLUDES = $(wildcard $(addsuffix /include, $(filter umdk-%, ${USEMODULE})))

multiple_targets: create_headers all

create_headers:
	echo $(SHELL)
	@modlist=(`echo $(UMDK-INCLUDES)`); \
	echo "/* DO NOT EDIT! FILE IS AUTO GENERATED */" > unwds-common/include/umdk-modules.h; \
	echo "#ifndef UMDK_MODULES_H_" >> unwds-common/include/umdk-modules.h; \
	echo -e "#define UMDK_MODULES_H_\n" >> unwds-common/include/umdk-modules.h; \
	for i in "$${modlist[@]}"; do \
	for k in `find ./$$i -type f -printf "%f\n"`; do echo "#include <$$k>" >> unwds-common/include/umdk-modules.h; done; \
	done; \
	printf "\nstatic const unwd_module_t modules[] = {\n" >> unwds-common/include/umdk-modules.h; \
	for k in "$${modlist[@]}"; do \
	name=`echo $$k | sed -e "s/^umdk-//" | sed -e "s/\/include$$//"`; \
	name_up=`echo $$name | tr '[:lower:]' '[:upper:]'`; \
	printf "{ UNWDS_$${name_up}_MODULE_ID, \"$${name}\", " >> unwds-common/include/umdk-modules.h; \
	if grep -q "umdk_$${name}_init" $$k/*; then printf "umdk_$${name}_init, "  >> unwds-common/include/umdk-modules.h; \
	else printf "NULL, " >> unwds-common/include/umdk-modules.h; fi; \
	if grep -q "umdk_$${name}_cmd" $$k/*; then printf "umdk_$${name}_cmd, " >> unwds-common/include/umdk-modules.h; \
	else printf "NULL, "  >> unwds-common/include/umdk-modules.h; fi; \
	if grep -q "umdk_$${name}_broadcast" $$k/*; then printf "umdk_$${name}_broadcast"  >> unwds-common/include/umdk-modules.h; \
	else printf "NULL"  >> unwds-common/include/umdk-modules.h; fi; \
	printf "},\n"  >> unwds-common/include/umdk-modules.h; \
	sed -i "s/#define _UMDK_MID_.*/#define _UMDK_MID_ UNWDS_$${name_up}_MODULE_ID/" umdk-$${name}/*.c; \
    sed -i "s/#define _UMDK_NAME_.*/#define _UMDK_NAME_ \"$${name}\"/" umdk-$${name}/*.c; \
	done; \
	printf "{ 0, \"\", NULL, NULL } };\n" >> unwds-common/include/umdk-modules.h; \
	echo -e "\n#endif" >> unwds-common/include/umdk-modules.h;

include $(RIOTBASE)/Makefile.base