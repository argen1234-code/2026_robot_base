const app = getApp()

const STORAGE_KEY = 'pure_gps_route_library_v1'
const MAX_POINTS = 10
const PI = Math.PI
const GCJ_A = 6378245.0
const GCJ_EE = 0.006693421622965943

const modeOptions = [
  { value: 1, label: 'GPS+ROS融合', command: 'GPS_ROS', url: '/pages/gps/gps' },
  { value: 2, label: '遥控模式', command: 'REMOTE', url: '/pages/control/control' },
  { value: 3, label: '室内导航', command: 'INDOOR', url: '/pages/indoor/indoor' },
  { value: 4, label: '纯GPS', command: 'GPS_ONLY', url: '/pages/puregps/puregps' }
]

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

Page({
  data: {
    state: app.globalData.state,
    modeOptions,
    switchingMode: 0,
    gpsValid: false,
    mapLongitude: 116.397428,
    mapLatitude: 39.90923,
    markers: [],
    points: [],
    adding: false,
    savedRoutes: [],
    sending: false,
    speed: 100
  },

  onLoad() {
    this.unsubscribe = app.globalData.api.onState(state => this.applyState(state))
    this.loadRouteLibrary()
  },

  onUnload() {
    if (this.unsubscribe) this.unsubscribe()
  },

  onShow() {
    const state = app.globalData.api.getState()
    if (!state.online || state.mode !== 4) {
      wx.showToast({ title: '请先切换到纯GPS模式', icon: 'none' })
      wx.switchTab({ url: '/pages/home/home' })
      return
    }
    this.applyState(state)
  },

  applyState(state) {
    const longitude = Number(state.longitude_car || 0)
    const latitude = Number(state.latitude_car || 0)
    const gpsValid = Boolean(state.gps_valid && longitude && latitude)
    const display = gpsValid
      ? wgs84ToGcj02(latitude, longitude)
      : { latitude: this.data.mapLatitude, longitude: this.data.mapLongitude }
    const confirmed = this.data.switchingMode && state.mode === this.data.switchingMode
    this.setData({
      state,
      gpsValid,
      mapLongitude: display.longitude,
      mapLatitude: display.latitude,
      markers: this.buildMarkers(gpsValid, latitude, longitude),
      switchingMode: confirmed ? 0 : this.data.switchingMode
    }, () => {
      if (confirmed && this._pendingModeUrl) {
        const url = this._pendingModeUrl
        this._pendingModeUrl = ''
        wx.switchTab({ url })
      }
    })
  },

  buildMarkers(gpsValid, latitude, longitude) {
    const markers = []
    if (gpsValid) {
      const robot = wgs84ToGcj02(latitude, longitude)
      markers.push({
        id: 1,
        latitude: robot.latitude,
        longitude: robot.longitude,
        width: 32,
        height: 32,
        callout: { content: '机器人', display: 'ALWAYS', padding: 6, borderRadius: 6, bgColor: '#1e3a8a', color: '#ffffff' }
      })
    }
    this.data.points.forEach((point, index) => {
      const display = wgs84ToGcj02(point.latitude, point.longitude)
      markers.push({
        id: index + 100,
        latitude: display.latitude,
        longitude: display.longitude,
        width: 28,
        height: 28,
        label: { content: `${index + 1}`, color: '#ffffff', bgColor: '#f59e0b', borderRadius: 12, padding: 4 }
      })
    })
    return markers
  },

  refreshMarkers() {
    this.setData({
      markers: this.buildMarkers(
        this.data.gpsValid,
        Number(this.data.state.latitude_car || 0),
        Number(this.data.state.longitude_car || 0))
    })
  },

  enterMode(event) {
    const mode = Number(event.currentTarget.dataset.mode)
    const option = modeOptions.find(item => item.value === mode)
    if (!option || !this.data.state.online) return
    if (this.data.state.mode === mode) return wx.switchTab({ url: option.url })
    this._pendingModeUrl = option.url
    this.setData({ switchingMode: mode })
    app.globalData.api.sendCommand(option.command).catch(error => {
      this._pendingModeUrl = ''
      this.setData({ switchingMode: 0 })
      wx.showToast({ title: error.message || '模式切换失败', icon: 'none' })
    })
  },

  toggleAdding() {
    this.setData({ adding: !this.data.adding })
  },

  onMapTap(event) {
    if (!this.data.adding || this.data.points.length >= MAX_POINTS) return
    const detail = event.detail || {}
    if (!Number.isFinite(Number(detail.latitude)) || !Number.isFinite(Number(detail.longitude))) return
    const point = gcj02ToWgs84(Number(detail.latitude), Number(detail.longitude))
    this.addPoint(point.latitude, point.longitude)
    this.setData({ adding: false })
  },

  recordCurrent() {
    if (!this.data.gpsValid) return wx.showToast({ title: '当前GPS定位无效', icon: 'none' })
    this.addPoint(Number(this.data.state.latitude_car), Number(this.data.state.longitude_car))
  },

  addPoint(latitude, longitude) {
    if (this.data.points.length >= MAX_POINTS) {
      return wx.showToast({ title: 'STM32最多支持10个点', icon: 'none' })
    }
    const point = {
      id: Date.now() + Math.random(),
      latitude,
      longitude,
      latitudeText: Number(latitude).toFixed(6),
      longitudeText: Number(longitude).toFixed(6)
    }
    this.setData({ points: this.data.points.concat([point]) }, () => this.refreshMarkers())
  },

  clearPoints() {
    this.setData({ points: [], adding: false }, () => this.refreshMarkers())
  },

  moveUp(event) { this.reorder(Number(event.currentTarget.dataset.index), -1) },
  moveDown(event) { this.reorder(Number(event.currentTarget.dataset.index), 1) },
  reorder(index, offset) {
    const target = index + offset
    if (target < 0 || target >= this.data.points.length) return
    const points = this.data.points.slice()
    const point = points.splice(index, 1)[0]
    points.splice(target, 0, point)
    this.setData({ points }, () => this.refreshMarkers())
  },
  removePoint(event) {
    const points = this.data.points.slice()
    points.splice(Number(event.currentTarget.dataset.index), 1)
    this.setData({ points }, () => this.refreshMarkers())
  },

  loadRouteLibrary() {
    const savedRoutes = wx.getStorageSync(STORAGE_KEY)
    this.setData({ savedRoutes: Array.isArray(savedRoutes) ? savedRoutes : [] })
  },

  saveCurrentRoute() {
    if (!this.data.points.length) return wx.showToast({ title: '请先采集路径点', icon: 'none' })
    wx.showModal({
      title: '保存路径数组',
      editable: true,
      placeholderText: `纯GPS路径 ${this.data.savedRoutes.length + 1}`,
      success: result => {
        if (!result.confirm) return
        const route = {
          id: Date.now(),
          name: (result.content || '').trim() || `纯GPS路径 ${this.data.savedRoutes.length + 1}`,
          createdAt: new Date().toLocaleString(),
          points: this.data.points.map(point => ({
            latitude: point.latitude,
            longitude: point.longitude,
            latitudeText: point.latitudeText,
            longitudeText: point.longitudeText
          }))
        }
        const savedRoutes = [route].concat(this.data.savedRoutes)
        wx.setStorageSync(STORAGE_KEY, savedRoutes)
        this.setData({ savedRoutes })
        wx.showToast({ title: '路径数组已保存', icon: 'success' })
      }
    })
  },

  loadSavedRoute(event) {
    const route = this.data.savedRoutes[Number(event.currentTarget.dataset.index)]
    if (!route) return
    const points = route.points.slice(0, MAX_POINTS).map((point, index) => ({
      id: Date.now() + index,
      latitude: Number(point.latitude),
      longitude: Number(point.longitude),
      latitudeText: Number(point.latitude).toFixed(6),
      longitudeText: Number(point.longitude).toFixed(6)
    }))
    this.setData({ points }, () => this.refreshMarkers())
  },

  deleteSavedRoute(event) {
    const savedRoutes = this.data.savedRoutes.slice()
    savedRoutes.splice(Number(event.currentTarget.dataset.index), 1)
    wx.setStorageSync(STORAGE_KEY, savedRoutes)
    this.setData({ savedRoutes })
  },

  speedChange(event) {
    const speed = Number(event.detail.value)
    this.setData({ speed })
    app.globalData.api.sendCommand('SPEED', { mode: 4, speed })
      .then(() => wx.showToast({ title: `纯GPS速度 ${speed}%`, icon: 'none' }))
      .catch(error => wx.showToast({ title: error.message || '速度设置失败', icon: 'none' }))
  },

  sendRouteToStm32() {
    if (this.data.sending) return
    const state = this.data.state
    const stm32ModeKnown = state.stm32_mode !== undefined && state.stm32_mode !== null
    if (!state.online || state.mode !== 4 || (stm32ModeKnown && Number(state.stm32_mode) !== 4)) {
      return wx.showToast({ title: '请先进入纯GPS模式', icon: 'none' })
    }
    if (!state.gps_valid || !state.mag_valid) {
      return wx.showToast({ title: 'GPS或磁力计无效，暂不能开始巡航', icon: 'none' })
    }
    if (!this.data.points.length) return wx.showToast({ title: '路径数组为空', icon: 'none' })
    const waypoints = this.data.points
      .map(point => ({ latitude: Number(point.latitude), longitude: Number(point.longitude) }))
      .filter(point => Number.isFinite(point.latitude) && Number.isFinite(point.longitude))
    if (!waypoints.length) return wx.showToast({ title: '路径点坐标无效', icon: 'none' })
    this.setData({ sending: true })
    app.globalData.api.sendCommand('GPS_ONLY_ROUTE_SET', {
      loop_enable: true,
      speed: this.data.speed,
      waypoints
    }).then(() => {
      this.setData({ sending: false })
      wx.showToast({ title: '路径已开始下发', icon: 'success' })
    }).catch(error => {
      this.setData({ sending: false })
      wx.showToast({ title: error.message || '路径下发失败', icon: 'none' })
    })
  },

  clearStm32Route() {
    app.globalData.api.sendCommand('GPS_ONLY_ROUTE_CLEAR')
      .then(() => wx.showToast({ title: '清空指令已发送', icon: 'none' }))
      .catch(error => wx.showToast({ title: error.message || '发送失败', icon: 'none' }))
  }
})
