const app = getApp()

const PI = Math.PI
const GCJ_A = 6378245.0
const GCJ_EE = 0.006693421622965943

function outsideChina(latitude, longitude) {
  return longitude < 72.004 || longitude > 137.8347 || latitude < 0.8293 || latitude > 55.8271
}

function transformLatitude(x, y) {
  let value = -100 + 2 * x + 3 * y + 0.2 * y * y + 0.1 * x * y + 0.2 * Math.sqrt(Math.abs(x))
  value += (20 * Math.sin(6 * x * PI) + 20 * Math.sin(2 * x * PI)) * 2 / 3
  value += (20 * Math.sin(y * PI) + 40 * Math.sin(y / 3 * PI)) * 2 / 3
  value += (160 * Math.sin(y / 12 * PI) + 320 * Math.sin(y * PI / 30)) * 2 / 3
  return value
}

function transformLongitude(x, y) {
  let value = 300 + x + 2 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * Math.sqrt(Math.abs(x))
  value += (20 * Math.sin(6 * x * PI) + 20 * Math.sin(2 * x * PI)) * 2 / 3
  value += (20 * Math.sin(x * PI) + 40 * Math.sin(x / 3 * PI)) * 2 / 3
  value += (150 * Math.sin(x / 12 * PI) + 300 * Math.sin(x / 30 * PI)) * 2 / 3
  return value
}

function wgs84ToGcj02(latitude, longitude) {
  if (outsideChina(latitude, longitude)) return { latitude, longitude }
  let dLat = transformLatitude(longitude - 105, latitude - 35)
  let dLon = transformLongitude(longitude - 105, latitude - 35)
  const radLat = latitude / 180 * PI
  let magic = Math.sin(radLat)
  magic = 1 - GCJ_EE * magic * magic
  const sqrtMagic = Math.sqrt(magic)
  dLat = dLat * 180 / ((GCJ_A * (1 - GCJ_EE)) / (magic * sqrtMagic) * PI)
  dLon = dLon * 180 / (GCJ_A / sqrtMagic * Math.cos(radLat) * PI)
  return { latitude: latitude + dLat, longitude: longitude + dLon }
}

function gcj02ToWgs84(latitude, longitude) {
  if (outsideChina(latitude, longitude)) return { latitude, longitude }
  const converted = wgs84ToGcj02(latitude, longitude)
  return {
    latitude: latitude * 2 - converted.latitude,
    longitude: longitude * 2 - converted.longitude
  }
}

const modeOptions = [
  { value: 1, label: 'GPS+ROS融合', command: 'GPS_ROS', url: '/pages/gps/gps' },
  { value: 2, label: '遥控模式', command: 'REMOTE', url: '/pages/control/control' },
  { value: 3, label: '室内导航', command: 'INDOOR', url: '/pages/indoor/indoor' },
  { value: 4, label: '纯GPS', command: 'GPS_ONLY', url: '/pages/puregps/puregps' }
]

Page({
  data: {
    state: app.globalData.state,
    modeOptions,
    switchingMode: 0,
    gpsValid: false,
    longitudeText: '0.000000',
    latitudeText: '0.000000',
    mapLongitude: 116.397428,
    mapLatitude: 39.90923,
    markers: [],
    rosMapImagePath: '',
    rosMapMeta: null,
    rosCanvasWidth: 300,
    rosCanvasHeight: 260,
    gpsWaypoints: [],
    addingGpsPoint: false,
    patrolMode: 'PING_PONG',
    dwellSeconds: 2,
    speed: 100
  },

  onLoad() {
    const api = app.globalData.api
    this.unsubscribe = api.onState(state => this.applyState(state))
    this.unsubscribeMap = api.onMap(payload => this.onRosMap(payload))
  },

  onReady() {
    wx.createSelectorQuery().in(this).select('.fusion-map-stage').boundingClientRect(rect => {
      if (!rect) return
      this.setData({
        rosCanvasWidth: Math.round(rect.width),
        rosCanvasHeight: Math.round(rect.height)
      }, () => this.drawRosMap())
    }).exec()
  },

  onUnload() {
    if (this.unsubscribe) this.unsubscribe()
    if (this.unsubscribeMap) this.unsubscribeMap()
    if (this.data.rosMapImagePath) {
      wx.getFileSystemManager().unlink({ filePath: this.data.rosMapImagePath, fail: () => {} })
    }
  },

  onShow() {
    const state = app.globalData.api.getState()
    if (!state.online || state.mode !== 1) {
      wx.showToast({ title: '请先切换到GPS模式', icon: 'none' })
      wx.switchTab({ url: '/pages/home/home' })
      return
    }
    this.applyState(state)
  },

  applyState(state) {
    const longitude = Number(state.longitude_car || 0)
    const latitude = Number(state.latitude_car || 0)
    const gpsValid = Boolean(state.gps_valid && longitude && latitude)
    const displayPosition = gpsValid
      ? wgs84ToGcj02(latitude, longitude)
      : { latitude: this.data.mapLatitude, longitude: this.data.mapLongitude }
    const markers = this.buildGpsMarkers(gpsValid, longitude, latitude)
    const confirmed = this.data.switchingMode && state.mode === this.data.switchingMode
    this.setData({
      state,
      gpsValid,
      longitudeText: longitude.toFixed(6),
      latitudeText: latitude.toFixed(6),
      mapLongitude: displayPosition.longitude,
      mapLatitude: displayPosition.latitude,
      markers,
      switchingMode: confirmed ? 0 : this.data.switchingMode
    }, () => {
      this.drawRosMap()
      if (confirmed && this._pendingModeUrl) {
        const url = this._pendingModeUrl
        this._pendingModeUrl = ''
        wx.switchTab({ url })
      }
    })
  },

  buildGpsMarkers(gpsValid, longitude, latitude) {
    const robotDisplay = wgs84ToGcj02(latitude, longitude)
    const markers = gpsValid ? [{
      id: 1,
      longitude: robotDisplay.longitude,
      latitude: robotDisplay.latitude,
      width: 32,
      height: 32,
      callout: {
        content: (this.data.state && this.data.state.robot_id) || 'robot_001',
        display: 'ALWAYS',
        padding: 6,
        borderRadius: 6,
        bgColor: '#1e3a8a',
        color: '#ffffff'
      }
    }] : []
    this.data.gpsWaypoints.forEach((point, index) => {
      const displayPoint = wgs84ToGcj02(point.latitude, point.longitude)
      markers.push({
        id: index + 100,
        longitude: displayPoint.longitude,
        latitude: displayPoint.latitude,
        width: 28,
        height: 28,
        label: {
          content: `${index + 1}`,
          color: '#ffffff',
          bgColor: '#f59e0b',
          borderRadius: 12,
          padding: 4
        }
      })
    })
    return markers
  },

  refreshGpsMarkers() {
    const longitude = Number(this.data.state.longitude_car || 0)
    const latitude = Number(this.data.state.latitude_car || 0)
    this.setData({
      markers: this.buildGpsMarkers(this.data.gpsValid, longitude, latitude)
    })
  },

  toggleGpsAdding() {
    this.setData({ addingGpsPoint: !this.data.addingGpsPoint })
  },

  onGpsMapTap(event) {
    if (!this.data.addingGpsPoint) return
    const detail = event.detail || {}
    const longitude = Number(detail.longitude)
    const latitude = Number(detail.latitude)
    if (!Number.isFinite(longitude) || !Number.isFinite(latitude)) return
    const wgs84Point = gcj02ToWgs84(latitude, longitude)
    this.addGpsWaypoint(wgs84Point.latitude, wgs84Point.longitude)
    this.setData({ addingGpsPoint: false })
  },

  addGpsWaypoint(latitude, longitude) {
    const point = {
      id: Date.now() + Math.random(),
      name: `GPS目标点 ${this.data.gpsWaypoints.length + 1}`,
      latitude: Number(latitude),
      longitude: Number(longitude),
      latitudeText: Number(latitude).toFixed(6),
      longitudeText: Number(longitude).toFixed(6)
    }
    this.setData({ gpsWaypoints: this.data.gpsWaypoints.concat([point]) }, () => {
      this.refreshGpsMarkers()
    })
  },

  recordCurrentGps() {
    if (!this.data.gpsValid) {
      return wx.showToast({ title: '当前GPS定位无效', icon: 'none' })
    }
    this.addGpsWaypoint(
      Number(this.data.state.latitude_car),
      Number(this.data.state.longitude_car))
  },

  clearGpsWaypoints() {
    this.setData({ gpsWaypoints: [], addingGpsPoint: false }, () => this.refreshGpsMarkers())
  },

  moveGpsPointUp(event) {
    this.reorderGpsPoint(Number(event.currentTarget.dataset.index), -1)
  },

  moveGpsPointDown(event) {
    this.reorderGpsPoint(Number(event.currentTarget.dataset.index), 1)
  },

  reorderGpsPoint(index, offset) {
    const target = index + offset
    if (target < 0 || target >= this.data.gpsWaypoints.length) return
    const points = this.data.gpsWaypoints.slice()
    const point = points.splice(index, 1)[0]
    points.splice(target, 0, point)
    this.setData({ gpsWaypoints: points }, () => this.refreshGpsMarkers())
  },

  removeGpsPoint(event) {
    const points = this.data.gpsWaypoints.slice()
    points.splice(Number(event.currentTarget.dataset.index), 1)
    this.setData({ gpsWaypoints: points }, () => this.refreshGpsMarkers())
  },

  setGpsPatrolMode(event) {
    this.setData({ patrolMode: event.currentTarget.dataset.mode })
  },

  gpsDwellChange(event) {
    this.setData({ dwellSeconds: Number(event.detail.value) })
  },

  gpsSpeedChange(event) {
    const speed = Number(event.detail.value)
    this.setData({ speed })
    app.globalData.api.sendCommand('SPEED', { mode: 1, speed })
      .then(() => wx.showToast({ title: `融合速度 ${speed}%`, icon: 'none' }))
      .catch(error => wx.showToast({ title: error.message || '速度设置失败', icon: 'none' }))
  },

  startGpsMission() {
    if (!this.data.state.online || this.data.state.mode !== 1) {
      return wx.showToast({ title: '请先进入GPS+ROS融合模式', icon: 'none' })
    }
    if (!this.data.gpsValid) {
      return wx.showToast({ title: 'GPS定位无效，不能开始', icon: 'none' })
    }
    if (!this.data.gpsWaypoints.length) {
      return wx.showToast({ title: '请先添加GPS目标点', icon: 'none' })
    }
    const waypoints = this.data.gpsWaypoints.map(point => ({
      latitude: point.latitude,
      longitude: point.longitude
    }))
    app.globalData.api.sendCommand('GPS_ROS_MISSION_START', {
      mission_id: `gps_ros_${Date.now()}`,
      patrol_mode: this.data.patrolMode,
      dwell_seconds: this.data.dwellSeconds,
      speed: this.data.speed,
      waypoints
    }).then(() => {
      wx.showToast({ title: '融合导航任务已发送', icon: 'success' })
    }).catch(error => wx.showToast({ title: error.message || '发送失败', icon: 'none' }))
  },

  cancelGpsMission() {
    app.globalData.api.sendCommand('GPS_ROS_MISSION_CANCEL')
      .then(() => wx.showToast({ title: '任务已取消', icon: 'none' }))
      .catch(error => wx.showToast({ title: error.message || '取消失败', icon: 'none' }))
  },

  onRosMap(payload) {
    if (!payload || !payload.png_base64) return
    const path = `${wx.env.USER_DATA_PATH}/fusion_map_${payload.revision || Date.now()}.png`
    const previousPath = this.data.rosMapImagePath
    wx.getFileSystemManager().writeFile({
      filePath: path,
      data: payload.png_base64,
      encoding: 'base64',
      success: () => {
        this.setData({ rosMapImagePath: path, rosMapMeta: payload }, () => {
          this.drawRosMap()
          if (previousPath && previousPath !== path) {
            wx.getFileSystemManager().unlink({ filePath: previousPath, fail: () => {} })
          }
        })
      },
      fail: error => console.error('融合导航地图保存失败', error)
    })
  },

  rosMapRenderRect() {
    const meta = this.data.rosMapMeta
    if (!meta) return null
    const scale = Math.min(this.data.rosCanvasWidth / meta.width, this.data.rosCanvasHeight / meta.height)
    const width = meta.width * scale
    const height = meta.height * scale
    return {
      scale,
      width,
      height,
      left: (this.data.rosCanvasWidth - width) / 2,
      top: (this.data.rosCanvasHeight - height) / 2
    }
  },

  rosWorldToCanvas(x, y) {
    const meta = this.data.rosMapMeta
    const rect = this.rosMapRenderRect()
    if (!meta || !rect) return null
    const dx = Number(x || 0) - Number(meta.origin_x || 0)
    const dy = Number(y || 0) - Number(meta.origin_y || 0)
    const originYaw = Number(meta.origin_yaw || 0)
    const cosYaw = Math.cos(originYaw)
    const sinYaw = Math.sin(originYaw)
    const localX = dx * cosYaw + dy * sinYaw
    const localY = -dx * sinYaw + dy * cosYaw
    return {
      x: rect.left + (localX / meta.resolution) * rect.scale,
      y: rect.top + (meta.height - localY / meta.resolution) * rect.scale
    }
  },

  drawRosMap() {
    if (!this.data.rosMapMeta) return
    const ctx = wx.createCanvasContext('fusionMapOverlay', this)
    ctx.clearRect(0, 0, this.data.rosCanvasWidth, this.data.rosCanvasHeight)
    const robot = this.rosWorldToCanvas(this.data.state.x, this.data.state.y)
    if (robot && this.data.state.online) {
      const yaw = Number(this.data.state.yaw || 0) * Math.PI / 180
      const screenYaw = yaw - Number(this.data.rosMapMeta.origin_yaw || 0)
      ctx.setFillStyle('#2563eb')
      ctx.beginPath()
      ctx.arc(robot.x, robot.y, 11, 0, Math.PI * 2)
      ctx.fill()
      ctx.setStrokeStyle('#172554')
      ctx.setLineWidth(4)
      ctx.beginPath()
      ctx.moveTo(robot.x, robot.y)
      ctx.lineTo(robot.x + Math.cos(-screenYaw) * 26, robot.y + Math.sin(-screenYaw) * 26)
      ctx.stroke()
    }
    ctx.draw()
  },

  enterMode(event) {
    const mode = Number(event.currentTarget.dataset.mode)
    const option = modeOptions.find(item => item.value === mode)
    if (!option) return
    if (!this.data.state.online) return wx.showToast({ title: '机器人离线', icon: 'none' })
    if (this.data.state.mode === mode) {
      wx.switchTab({ url: option.url })
      return
    }
    this._pendingModeUrl = option.url
    this.setData({ switchingMode: mode })
    app.globalData.api.sendCommand(option.command).then(() => {
      wx.showToast({ title: '等待机器人确认模式', icon: 'none' })
    }).catch(error => {
      this._pendingModeUrl = ''
      this.setData({ switchingMode: 0 })
      wx.showToast({ title: error.message || '模式切换失败', icon: 'none' })
    })
  }
})
