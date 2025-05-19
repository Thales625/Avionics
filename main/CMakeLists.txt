set(COMPONENT_SRCS "main.cpp" "mpu6050.cpp" "wifi.c" "udp_trans.c" "web_server.c" "web_client.c")
set(COMPONENT_ADD_INCLUDEDIRS ".")
set(COMPONENT_EMBED_FILES "../html/error.html"
	"../html/main.js"
	"../html/root.html"
	"../html/main.css"
	"../html/favicon.ico")

register_component()