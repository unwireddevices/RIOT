DIRS += $(dir $(wildcard $(addsuffix /Makefile, ${USEMODULE})))

multiple_targets: create_headers all

create_headers:
	@echo "/* DO NOT EDIT! FILE IS AUTO GENERATED */" > unwds-common/include/umdk-modules.h; \
	echo "#ifndef UMDK_MODULES_H_" >> unwds-common/include/umdk-modules.h; \
	echo -e "#define UMDK_MODULES_H_\n" >> unwds-common/include/umdk-modules.h; \
	for i in `find . -path "./umdk*/include/*.h" -printf "%f\n"`; do echo "#include <$$i>" >> unwds-common/include/umdk-modules.h; done; \
	echo -e "\n#endif" >> unwds-common/include/umdk-modules.h;

include $(RIOTBASE)/Makefile.base