DIRS += $(dir $(wildcard $(addsuffix /Makefile, ${USEMODULE})))
INCLUDES += $(addprefix -I../../unwired-modules/, $(wildcard $(addsuffix /include, $(filter umdk-%, ${USEMODULE}))))
CFLAGS += $(addprefix -D, $(subst umdk-, umdk_, $(filter umdk-%, ${USEMODULE})))
UMDK-INCLUDES = $(wildcard $(addsuffix /include, $(filter umdk-%, ${USEMODULE})))

multiple_targets: create_headers all

create_headers:
	@echo "/* DO NOT EDIT! FILE IS AUTO GENERATED */" > unwds-common/include/umdk-modules.h; \
	echo "#ifndef UMDK_MODULES_H_" >> unwds-common/include/umdk-modules.h; \
	echo -e "#define UMDK_MODULES_H_\n" >> unwds-common/include/umdk-modules.h; \
	for i in '$(UMDK-INCLUDES)'; do \
	for k in `find ./$$i -type f -printf "%f\n"`; do echo "#include <$$k>" >> unwds-common/include/umdk-modules.h; done; \
	done; \
	echo -e "\n#endif" >> unwds-common/include/umdk-modules.h;

include $(RIOTBASE)/Makefile.base