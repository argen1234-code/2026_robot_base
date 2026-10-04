# Robot WeChat Mini Program

The mini program connects to the Jetson ROS MQTT bridge over EMQX WebSocket TLS.

1. Create a dedicated MQTT username/password for the mini program in EMQX Cloud (separate from Jetson and from the dashboard login).
2. Import this project in WeChat DevTools. On the **Status** tab, enter that MQTT username/password and select **Save and connect**. The app stores credentials in local WeChat storage on that device; they are not embedded in Git or sent to the management API. Clear them from the same page on a shared device. Local storage is not hardware-backed secure storage: a distributed mini program with a shared MQTT account can expose that account to users. Restrict its publish permission to the command topic and rotate it if compromised.
3. In the WeChat Mini Program admin console, add `wss://i6130f30.ala.cn-hangzhou.emqxsl.cn` under **Development Management -> Development Settings -> Server Domains -> socket legal domains**. DevTools' `urlCheck: false` only affects local debugging, not production devices.
4. The mini program uses WebSocket TLS port `8084` and path `/mqtt`. If the WeChat platform rejects a nonstandard port, put a WSS reverse proxy on 443 and change `BROKER_URL` in `utils/api.js` accordingly.

The command topic is `/k1ck5t83zdZ/test/user/get`. The Jetson publishes robot state,
map, path, and mission status on the corresponding `robot`, `map`, `path`, and
`mission` topics. The current Jetson implementation supports remote control and
one-shot indoor Nav2 missions; GPS pages are retained as source files but are not
shown in the default tab bar until a GPS ROS backend is added.
The MQTT login does not use the EMQX dashboard email/password or management API key.
