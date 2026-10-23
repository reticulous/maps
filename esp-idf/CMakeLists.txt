file(GLOB MAPS_SRCS "src/*.cpp")

include(${CMAKE_CURRENT_LIST_DIR}/spangap_requires.cmake)
# Conditional sources: esp-idf/conditional/<name>/ is compiled only when <name>
# is staged. The on-device map viewer lives in conditional/spangap-lcd/ and
# registers via the when:-gated mapsLcdRegister hook — no #if anywhere.
idf_component_register(
    SRCS
        ${MAPS_SRCS}
        ${SPANGAP_CONDITIONAL_SRCS}
    INCLUDE_DIRS "include" ${SPANGAP_CONDITIONAL_INCLUDE_DIRS}
    REQUIRES
        ${SPANGAP_REQUIRES}
    PRIV_REQUIRES
        espressif__esp_jpeg
        json
)
