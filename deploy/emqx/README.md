# EMQX access control

The unified repository uses two separate MQTT users. Replace the placeholders
in `acl-rules.json` with the actual usernames created in EMQX Cloud. Never put
passwords or App Secrets in this repository.

In EMQX Cloud, enable **Access Control -> Authorization -> Built-in Database**
and add the ten rules from `acl-rules.json`:

- Jetson subscribes to `get` and publishes `robot`, `map`, `path`, `mission`.
- WeChat publishes `get` and subscribes to `robot`, `map`, `path`, `mission`.

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
the dashboard first. Do not grant wildcard publish access to the mini-program.
