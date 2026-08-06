# Only the active role's dashboard is embedded, so the unused page costs nothing.
if(CONFIG_ROLE_RECEIVER)
    set(embed_page "www/receiver.html")
else()
    set(embed_page "www/sender.html")
endif()

idf_component_register(
    SRCS
        "main.c"
        "link.c"
        "net.c"
        "webserver.c"
        "sender.c"
        "receiver.c"
        "battery.c"
        "storage.c"
        "powerload.c"
    INCLUDE_DIRS "."
    EMBED_FILES ${embed_page}
    PRIV_REQUIRES
        sx126x
        nvs_flash
        esp_wifi
        esp_event
        esp_netif
        esp_http_server
        spiffs
        esp_adc
        esp_timer
)
