SHELL := /bin/bash # Use bash syntax

UMDK-MODULES-LIST = $(RIOTBASE)/unwired-modules/include/umdk-modules.h
UMDK-INCLUDES = $(wildcard $(addsuffix /include, $(UMDK_MODULES_ENABLED)))

multiple_targets: create_headers all

create_headers:
	@modlist=(`echo $(UMDK-INCLUDES)`); \
	echo "/* DO NOT EDIT! FILE IS AUTO GENERATED */" > $(UMDK-MODULES-LIST); \
	echo "#ifndef UMDK_MODULES_H_" >> $(UMDK-MODULES-LIST); \
	echo -e "#define UMDK_MODULES_H_\n" >> $(UMDK-MODULES-LIST); \
	for i in "$${modlist[@]}"; do \
	for k in `find ./$$i -type f -printf "%f\n"`; do echo "#include <$$k>" >> $(UMDK-MODULES-LIST); done; \
	done; \
	printf "\nstatic const unwd_module_t modules[] = {\n" >> $(UMDK-MODULES-LIST); \
	for k in "$${modlist[@]}"; do \
	name=`echo $$k | sed -e "s/^umdk-//" | sed -e "s/\/include$$//"`; \
	name_up=`echo $$name | tr '[:lower:]' '[:upper:]'`; \
	printf "{ UNWDS_$${name_up}_MODULE_ID, \"$${name}\", " >> $(UMDK-MODULES-LIST); \
	if grep -q "umdk_$${name}_init" $$k/*; then printf "umdk_$${name}_init, "  >> $(UMDK-MODULES-LIST); \
	else printf "NULL, " >> $(UMDK-MODULES-LIST); fi; \
	if grep -q "umdk_$${name}_cmd" $$k/*; then printf "umdk_$${name}_cmd, " >> $(UMDK-MODULES-LIST); \
	else printf "NULL, "  >> $(UMDK-MODULES-LIST); fi; \
	if grep -q "umdk_$${name}_broadcast" $$k/*; then printf "umdk_$${name}_broadcast"  >> $(UMDK-MODULES-LIST); \
	else printf "NULL"  >> $(UMDK-MODULES-LIST); fi; \
	printf "},\n"  >> $(UMDK-MODULES-LIST); \
	sed -i "s/#define _UMDK_MID_.*/#define _UMDK_MID_ UNWDS_$${name_up}_MODULE_ID/" umdk-$${name}/*.c; \
    sed -i "s/#define _UMDK_NAME_.*/#define _UMDK_NAME_ \"$${name}\"/" umdk-$${name}/*.c; \
	done; \
	printf "{ 0, \"\", NULL, NULL, NULL } };\n" >> $(UMDK-MODULES-LIST); \
	echo -e "\n#endif" >> $(UMDK-MODULES-LIST);

include $(RIOTBASE)/Makefile.base