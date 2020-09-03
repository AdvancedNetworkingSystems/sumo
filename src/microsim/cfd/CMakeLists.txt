set(microsim_cfd_STAT_SRCS
        Record.cpp
        Record.h
        Cfd.cpp
        Cfd.h
)

add_library(microsim_cfd STATIC ${microsim_cfd_STAT_SRCS})
set_property(TARGET microsim_cfd PROPERTY PROJECT_LABEL "z_microsim_cfd")
