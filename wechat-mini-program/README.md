# Robot WeChat Mini Program

This branch connects to the Jetson ROS MQTT bridge over EMQX WebSocket TLS.

1. Copy `utils/mqtt-config.js` to `utils/mqtt-config.local.js`.
2. Set the MQTT username and password created in EMQX. The local file is ignored by Git.
3. In WeChat DevTools, import this project and enable the WebSocket domain:
   `wss://i6130f30.ala.cn-hangzhou.emqxsl.cn`.
4. The mini program uses WebSocket TLS port `8084` and path `/mqtt`.

The command topic is `/k1ck5t83zdZ/test/user/get`. The Jetson publishes robot state,
map, path, and mission status on the corresponding `robot`, `map`, `path`, and
`mission` topics. The current Jetson implementation supports remote control and
one-shot indoor Nav2 missions; GPS pages are retained as source files but are not
shown in the default tab bar until a GPS ROS backend is added.
