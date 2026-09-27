# Komponente "main": DSP, Gruppendekoder und Anwendung
idf_component_register(
    SRCS "main.c" "rds_dsp.c" "rds_group.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_adc esp_timer freertos log
)
