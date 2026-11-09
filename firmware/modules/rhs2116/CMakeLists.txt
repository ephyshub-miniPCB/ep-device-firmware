# Intan RHS2116 driver (out-of-tree Zephyr module, see README.md)

if(CONFIG_RHS2116)
  zephyr_include_directories(include)

  zephyr_library_named(rhs2116)
  zephyr_library_sources(
    src/rhs2116.c
    src/rhs2116_ctrl.c
    src/rhs2116_transport.c
    src/rhs2116_stream.c
  )
  zephyr_library_sources_ifdef(CONFIG_RHS2116_EMUL src/rhs2116_emul.c)
endif()
