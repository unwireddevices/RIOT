MODULE=unwds_networking_common

# ifeq (,$(filter -DUNWDS_ROOT,$(CFLAGS)))
  SRC := $(filter-out root_node.c,$(wildcard *.c))
# endif

# ifeq (,$(filter -DUNWDS_DAG,$(CFLAGS)))
  # SRC := $(filter-out dag_node.c,$(wildcard *.c))
# endif

include $(RIOTBASE)/Makefile.base
