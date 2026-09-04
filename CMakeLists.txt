# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.0)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
# diffdrive_pico_firmware — firmware bas niveau d'une base differentielle ROS 2
# Cible : Raspberry Pi Pico 2 (RP2350), pico-sdk 2.3.0

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# == NE PAS MODIFIER : bloc requis par l'extension VS Code Raspberry Pi Pico ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.0)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ============================================================================

set(PICO_BOARD pico2 CACHE STRING "Board type")

include(pico_sdk_import.cmake)

project(diffdrive_pico_firmware C CXX ASM)

pico_sdk_init()

add_executable(diffdrive_pico_firmware
        src/main.c
        src/log.c
        src/encoders.c
        src/motors.c
        src/bno055.c
        src/pid.c
        src/protocol.c
        src/testmodes.c
)

pico_set_program_name(diffdrive_pico_firmware "diffdrive_pico_firmware")
pico_set_program_version(diffdrive_pico_firmware "1.0.0")

# Decodeur de quadrature x4 : genere quadrature.pio.h
pico_generate_pio_header(diffdrive_pico_firmware ${CMAKE_CURRENT_LIST_DIR}/pio/quadrature.pio)

# ---------------------------------------------------------------------------
# stdio — REGLE CRITIQUE
# ---------------------------------------------------------------------------
# stdio est le protocole robot, sur USB CDC, et rien d'autre. La console UART
# est desactivee : les logs de debug passent par uart_write_blocking sur uart0
# (cf. log.h), ce qui court-circuite completement stdio.
pico_enable_stdio_uart(diffdrive_pico_firmware 0)
pico_enable_stdio_usb(diffdrive_pico_firmware 1)

target_compile_definitions(diffdrive_pico_firmware PRIVATE
        # Sans cela le SDK traduit \n en \r\n et corrompt le framing du
        # protocole. Non negociable.
        PICO_STDIO_DEFAULT_CRLF=0

        # Ecriture stdout bornee : si le Pi ne lit pas son port, les octets
        # sont abandonnes au bout de 1 ms au lieu des 500 ms par defaut. La
        # boucle 50 Hz ne peut donc jamais se faire bloquer par l'hote.
        PICO_STDIO_USB_STDOUT_TIMEOUT_US=1000

        # Ne jamais attendre l'ouverture du port par l'hote au demarrage :
        # le robot doit etre operationnel meme si personne n'ecoute.
        PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS=0

        # Descripteur USB. Le SDK lit USBD_PRODUCT / USBD_MANUFACTURER dans
        # pico_stdio_usb/stdio_usb_descriptors.c ; sans cela le peripherique
        # s'annonce "Raspberry Pi Pico", indistinguable de n'importe quelle
        # autre carte. Utile pour une regle udev cote Pi.
        USBD_PRODUCT="diffdrive_pico_firmware"
        USBD_MANUFACTURER="MatthieuGrr"
)

# Les en-tetes du firmware vivent dans include/. L'en-tete genere par pioasm
# (quadrature.pio.h) est ajoute au chemin d'inclusion par
# pico_generate_pio_header ci-dessus, il n'a pas a figurer ici.
target_include_directories(diffdrive_pico_firmware PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/include
)

# ---------------------------------------------------------------------------
# Bibliotheques
# ---------------------------------------------------------------------------
# hardware_pwm n'est PAS ajoute par l'assistant de creation de projet : sans
# cette ligne, la compilation echoue sur pwm_init & co.
target_link_libraries(diffdrive_pico_firmware
        pico_stdlib
        hardware_pwm
        hardware_pio
        hardware_i2c
        hardware_uart
        hardware_timer
        hardware_watchdog
        hardware_clocks
        hardware_gpio
        hardware_sync
)

# uf2 + elf + bin + carte des symboles
pico_add_extra_outputs(diffdrive_pico_firmware)
