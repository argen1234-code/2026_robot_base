const mqtt = require('./mqtt.wx.js')
let localConfig = { username: '', password: '' }
try { localConfig = require('./mqtt-config.local.js') } catch (error) {}

const BROKER_URL = 'wxs://i6130f30.ala.cn-hangzhou.emqxsl.cn:8084/mqtt'
const MQTT_OPTIONS = {
  clientId: `wechat_001_${Date.now()}_${Math.random().toString(16).slice(2, 8)}`,
  username: localConfig.username || '',
  password: localConfig.password || '',
  clean: true,
  connectTimeout: 10000,
  reconnectPeriod: 3000,
  keepalive: 30
}

const TOPICS = {
  command: '/k1ck5t83zdZ/test/user/get',
  state: '/k1ck5t83zdZ/test/user/robot',
  gps: '/k1ck5t83zdZ/test/user/esp8266duan',
  map: '/k1ck5t83zdZ/test/user/map',
  path: '/k1ck5t83zdZ/test/user/path',
  mission: '/k1ck5t83zdZ/test/user/mission'
}

const state = {
  robot_id: 'robot_001',
  online: false,
  mqtt_connected: false,
  mqtt_status: '未连接',
  mqtt_error: '',
  mode: 1,
  x: 0,
  y: 0,
  yaw: 0,
  longitude_car: 0,
  latitude_car: 0,
  navigation_state: 'UNKNOWN',
  lidar_ok: false,
  stm32_ok: false,
  gps_valid: false,
  mag_valid: false,
  gps_ros_state: {},
  gps_route_total: 0,
  error_code: 0,
  timestamp: 0,
  last_message_at: 0
}

let client = null
const listeners = []
const mapListeners = []
const pathListeners = []
const missionListeners = []
let latestMap = null
let latestPath = null
let latestMission = null
const COMMAND_PUBLISH_TIMEOUT = 8000

function snapshot() {
  return Object.assign({}, state)
}

function notify() {
  const current = snapshot()
  listeners.forEach(listener => {
    try { listener(current) } catch (error) { console.error('state listener error', error) }
  })
}

function mergeState(data) {
  Object.keys(data || {}).forEach(key => {
    if (data[key] !== undefined && data[key] !== null) state[key] = data[key]
  })
  if (data && data.gps_ros_state) {
    state.navigation_state = data.gps_ros_state.state || state.navigation_state
  }
  if (data && data.stm32_mode !== undefined && data.stm32_mode !== null) {
    state.stm32_ok = true
  }
  state.online = true
  state.last_message_at = Date.now()
  notify()
}

function parseMessage(topic, rawPayload) {
  if (topic === TOPICS.map || topic === TOPICS.path || topic === TOPICS.mission) {
    let payload
    try { payload = JSON.parse(rawPayload.toString()) } catch (error) {
      console.warn('室内导航消息不是 JSON', topic, error)
      return
    }
    if (topic === TOPICS.map) latestMap = payload
    else if (topic === TOPICS.path) latestPath = payload
    else latestMission = payload
    const targets = topic === TOPICS.map ? mapListeners : (topic === TOPICS.path ? pathListeners : missionListeners)
    targets.forEach(listener => {
      try { listener(payload) } catch (error) { console.error('室内消息监听器错误', error) }
    })
    return
  }

  let data
  try {
    data = JSON.parse(rawPayload.toString())
  } catch (error) {
    console.warn('MQTT 非 JSON 消息', topic, rawPayload.toString())
    return
  }

  if (topic === TOPICS.state) {
    mergeState(data)
  } else if (topic === TOPICS.gps) {
    mergeState({
      longitude_car: Number(data.longitude_car || 0),
      latitude_car: Number(data.latitude_car || 0),
      gps_valid: Number(data.longitude_car || 0) !== 0 || Number(data.latitude_car || 0) !== 0,
      timestamp: data.timestamp || state.timestamp
    })
  }
}

function connect() {
  if (client) {
    if (!state.mqtt_connected) {
      state.mqtt_status = '正在重连'
      state.mqtt_error = ''
      notify()
      try { client.reconnect() } catch (error) { console.error('MQTT 重连失败', error) }
    }
    return client
  }

  state.mqtt_status = '正在连接'
  state.mqtt_error = ''
  notify()

  console.log('MQTT 开始连接', BROKER_URL, MQTT_OPTIONS.clientId)
  try {
    client = mqtt.connect(BROKER_URL, MQTT_OPTIONS)
  } catch (error) {
    state.mqtt_status = '连接创建失败'
    state.mqtt_error = error.message || String(error)
    console.error('MQTT 创建连接失败', error)
    notify()
    return null
  }
  client.on('connect', () => {
    console.log('MQTT 已连接')
    state.mqtt_connected = true
    state.mqtt_status = '已连接，等待机器人数据'
    state.mqtt_error = ''
    client.subscribe([TOPICS.state, TOPICS.gps, TOPICS.map, TOPICS.path, TOPICS.mission], { qos: 1 }, error => {
      if (error) {
        state.mqtt_error = `订阅失败：${error.message || error}`
        console.error('MQTT 订阅失败', error)
      } else {
        console.log('MQTT 订阅成功', TOPICS.state, TOPICS.gps, TOPICS.map, TOPICS.path, TOPICS.mission)
      }
      notify()
    })
    notify()
  })
  client.on('message', parseMessage)
  client.on('reconnect', () => {
    console.log('MQTT 正在重连')
    state.mqtt_connected = false
    state.mqtt_status = '正在重连'
    notify()
  })
  client.on('close', () => {
    console.warn('MQTT 连接关闭')
    state.mqtt_connected = false
    state.online = false
    state.mqtt_status = '连接已关闭'
    notify()
  })
  client.on('offline', () => {
    console.warn('MQTT 离线')
    state.mqtt_connected = false
    state.online = false
    state.mqtt_status = 'MQTT 离线'
    notify()
  })
  client.on('error', error => {
    console.error('MQTT 连接错误', error)
    state.mqtt_connected = false
    state.mqtt_status = '连接错误'
    state.mqtt_error = error.message || String(error)
    notify()
  })

  return client
}

function sendCommand(command, extra) {
  return new Promise((resolve, reject) => {
    if (!client || !state.mqtt_connected) {
      reject(new Error('MQTT 未连接'))
      return
    }

    const payload = Object.assign({
      request_id: `${Date.now()}-${Math.random().toString(16).slice(2)}`,
      robot_id: state.robot_id,
      command,
      timestamp: Date.now()
    }, extra || {})

    let settled = false
    const finish = (error, result) => {
      if (settled) return
      settled = true
      clearTimeout(timeout)
      if (error) reject(error)
      else resolve(result)
    }
    const timeout = setTimeout(() => {
      finish(new Error('MQTT 指令发送超时，请检查小程序 MQTT 连接'))
    }, COMMAND_PUBLISH_TIMEOUT)

    try {
      client.publish(TOPICS.command, JSON.stringify(payload), { qos: 1 }, error => {
        if (error) finish(error)
        else finish(null, { success: true, command, message: 'published' })
      })
    } catch (error) {
      finish(error)
    }
  })
}

function onState(listener) {
  listeners.push(listener)
  listener(snapshot())
  return () => {
    const index = listeners.indexOf(listener)
    if (index >= 0) listeners.splice(index, 1)
  }
}

function addListener(list, listener, latest) {
  list.push(listener)
  if (latest) {
    try { listener(latest) } catch (error) { console.error('缓存消息监听器错误', error) }
  }
  return () => {
    const index = list.indexOf(listener)
    if (index >= 0) list.splice(index, 1)
  }
}

setInterval(() => {
  if (state.online && Date.now() - state.last_message_at > 5000) {
    state.online = false
    notify()
  }
}, 1000)

module.exports = {
  connect,
  sendCommand,
  onState,
  onMap: listener => addListener(mapListeners, listener, latestMap),
  onPath: listener => addListener(pathListeners, listener, latestPath),
  onMission: listener => addListener(missionListeners, listener, latestMission),
  getState: snapshot,
  reconnect: connect,
  TOPICS
}
