# EMQX access control

The unified repository uses two separate MQTT users. Create one user for the
Jetson bridge and one for the WeChat mini program. Never put their passwords or
an EMQX App Secret in this repository.

Use a stable pair of usernames, for example `robot_001_jetson` and
`robot_001_wechat`; if you choose different names, use those names consistently
in the dashboard, Jetson environment, and `acl-rules.json`.

## Configure it in EMQX Cloud

The dashboard labels vary slightly between EMQX Cloud plans, but the sequence is
the same:

1. Open the deployment and go to **Access Control -> Authentication**. Create a
   **Password-Based** authenticator with **Built-in Database** as its data
   source, enable it, and save it.
2. Open the authenticator's **Users** page and add `robot_001_jetson` and
   `robot_001_wechat`. Give them two different long random passwords. These are
   MQTT credentials; do not use the Cloud account email/password or an API key.
3. Go to **Access Control -> Authorization**, enable authorization, and add the
   **Built-in Database** source. Set the default authorization result to
   **Deny**. If several sources are shown, put the built-in database before any
   broad allow source.
4. Add the ten rules in [`acl-rules.json`](acl-rules.json), replacing its
   username placeholders with the two users you created. Select the exact action (**Publish** or
   **Subscribe**), permission **Allow**, and the exact topic filter. Set the
   username condition on every rule. Do not use `#` or `+` for the WeChat
   publish permission.
5. Save the source and use the dashboard's client/connection view to confirm
   that the two clients authenticate. A successful login alone does not grant
   topic access; authorization must also match.

The Jetson must use the first username and can subscribe only to `get`; it can
publish robot state, map, path, and mission. The mini program must use the
second username; it can publish commands only to `get` and subscribe to the
four robot topics.

The App ID/App Secret from an EMQX API key are management API credentials, not
MQTT credentials. The management API base URL is:

```text
https://i6130f30.ala.cn-hangzhou.emqxsl.cn:8443/api/v5
```

Use the App ID and App Secret as HTTP Basic credentials. For each object in the
JSON file, call:

```text
POST /api/v5/authorization/sources/built_in_database/rules
```

Example (values are intentionally supplied through environment variables):

```bash
curl --fail --silent --show-error \
  -u "$EMQX_API_KEY:$EMQX_API_SECRET" \
  -H 'content-type: application/json' \
  -X POST \
  'https://i6130f30.ala.cn-hangzhou.emqxsl.cn:8443/api/v5/authorization/sources/built_in_database/rules' \
  -d '{"username":"JETSON_MQTT_USERNAME","permission":"allow","action":"subscribe","topic":"/k1ck5t83zdZ/test/user/get"}'
```

If this endpoint returns `404`, enable the built-in authorization database in
the dashboard first. Do not grant wildcard publish access to the mini program.

## Test the two identities

From a machine with `mosquitto-clients`, replace the placeholders with the
corresponding MQTT credentials. The Jetson identity should receive commands and
publish state; the WeChat identity should receive state and publish commands.

```bash
export EMQX_HOST='i6130f30.ala.cn-hangzhou.emqxsl.cn'
export MQTT_USER='robot_001_wechat'
export MQTT_PASSWORD='replace-with-the-wechat-password'

mosquitto_sub --cafile /etc/ssl/certs/ca-certificates.crt \
  -h "$EMQX_HOST" -p 8883 -u "$MQTT_USER" -P "$MQTT_PASSWORD" \
  -t '/k1ck5t83zdZ/test/user/robot' -v
```

In another terminal, publish a harmless `STOP` command with the same identity:

```bash
mosquitto_pub --cafile /etc/ssl/certs/ca-certificates.crt \
  -h "$EMQX_HOST" -p 8883 -u "$MQTT_USER" -P "$MQTT_PASSWORD" \
  -t '/k1ck5t83zdZ/test/user/get' \
  -m '{"request_id":"acl-check-1","robot_id":"robot_001","command":"STOP","timestamp":0}'
```

The command is accepted by ACL before it reaches a robot. Do not publish motion
commands during an unattended test; the `STOP` payload is intentional.
