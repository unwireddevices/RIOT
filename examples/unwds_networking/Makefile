MODULE=unwds_networking_common

ifeq (,$(filter -DUNWDS_ROOT,$(CFLAGS)))
	ifneq (-DUNWDS_DAG,$(filter -DUNWDS_DAG,$(CFLAGS)))
		SRC := $(filter-out dag_node.c,$(wildcard *.c))
	endif
endif

# ifeq (-DUNWDS_DAG,$(filter -DUNWDS_DAG,$(CFLAGS)))
	# ifneq (-DUNWDS_ROOT,$(filter -DUNWDS_ROOT,$(CFLAGS)))
		# SRC := $(filter-out root_node.c,$(wildcard *.c))
	# endif
# endif

include $(RIOTBASE)/Makefile.base
