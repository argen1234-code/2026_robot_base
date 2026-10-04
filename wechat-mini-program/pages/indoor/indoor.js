const app = getApp()
const modeOptions = [
  { value: 2, label: '遥控模式', command: 'REMOTE', url: '/pages/control/control' },
  { value: 3, label: '室内导航', command: 'INDOOR', url: '/pages/indoor/indoor' }
]

Page({
  data: {
    state: app.globalData.state,
    imagePath: '',
    mapRevision: 0,
    mapMeta: null,
    pathPoints: [],
    waypoints: [],
    adding: false,
    patrolMode: 'ONCE',
    dwellSeconds: 2,
    speed: 100,
    mission: { state: '', message: '' },
    canvasWidth: 300,
    canvasHeight: 300,
    mapScale: 1,
    mapOffsetX: 0,
    mapOffsetY: 0,
    modeOptions,
    switchingMode: 0
  },

  onLoad() {
    const api = app.globalData.api
    this._unsubState = api.onState(state => this.applyState(state))
    this._unsubMap = api.onMap(payload => this.onMap(payload))
    this._unsubPath = api.onPath(payload => {
      this.setData({ pathPoints: payload.points || [] })
      this.drawOverlay()
    })
    this._unsubMission = api.onMission(payload => this.onMission(payload))
  },

  onReady() {
    this.refreshStageRect()
  },

  onPageScroll() {
    this.refreshStageRect()
  },

  refreshStageRect() {
    wx.createSelectorQuery().in(this).select('.map-stage').boundingClientRect(rect => {
      if (!rect) return
      this._stageRect = rect
      this.setData({ canvasWidth: Math.round(rect.width), canvasHeight: Math.round(rect.height) }, () => this.drawOverlay())
    }).exec()
  },

  onShow() {
    const state = app.globalData.api.getState()
    if (!state.online || state.mode !== 3) {
      wx.showToast({ title: '请先切换到室内导航模式', icon: 'none' })
      wx.switchTab({ url: '/pages/home/home' })
      return
    }
    this.applyState(state)
  },

  applyState(state) {
    const confirmed = this.data.switchingMode && state.mode === this.data.switchingMode
    this.setData({
      state,
      switchingMode: confirmed ? 0 : this.data.switchingMode
    }, () => {
      this.drawOverlay()
      if (confirmed && this._pendingModeUrl) {
        const url = this._pendingModeUrl
        this._pendingModeUrl = ''
        wx.switchTab({ url })
      }
    })
  },

  onUnload() {
    if (this._unsubState) this._unsubState()
    if (this._unsubMap) this._unsubMap()
    if (this._unsubPath) this._unsubPath()
    if (this._unsubMission) this._unsubMission()
  },

  onMap(payload) {
    if (!payload || !payload.png_base64) return
    const path = `${wx.env.USER_DATA_PATH}/indoor_map_${payload.revision}.png`
    const previousPath = this.data.imagePath
    wx.getFileSystemManager().writeFile({
      filePath: path,
      data: payload.png_base64,
      encoding: 'base64',
      success: () => {
        this.setData({ imagePath: path, mapRevision: payload.revision, mapMeta: payload }, () => {
          this.drawOverlay()
          if (previousPath && previousPath !== path) {
            wx.getFileSystemManager().unlink({ filePath: previousPath, fail: () => {} })
          }
        })
      },
      fail: error => wx.showToast({ title: `地图保存失败：${error.errMsg || ''}`, icon: 'none' })
    })
  },

  onMission(payload) {
    this.setData({ mission: payload })
    if (payload.state === 'RECORDED_POINT' && payload.point) {
      const point = Object.assign({ id: Date.now(), name: `记录点 ${this.data.waypoints.length + 1}` }, payload.point)
      this.setData({ waypoints: this.data.waypoints.concat([point]), adding: false }, () => this.drawOverlay())
    }
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
  },

  toggleAdding() { this.setData({ adding: !this.data.adding }) },

  onMapScale(event) {
    const scale = Number(event.detail.scale || 1)
    this._mapScale = Math.max(1, Math.min(4, scale))
    this._lastMapMoveAt = Date.now()
  },

  onMapMove(event) {
    if (event.detail && event.detail.source === 'touch') {
      this._lastMapMoveAt = Date.now()
    }
  },

  zoomIn() { this.zoomBy(0.5) },
  zoomOut() { this.zoomBy(-0.5) },
  zoomBy(delta) {
    const current = this._mapScale || this.data.mapScale || 1
    const next = Math.max(1, Math.min(4, Math.round((current + delta) * 10) / 10))
    this._mapScale = next
    this.setData({ mapScale: next })
  },
  resetZoom() {
    this._mapScale = 1
    this.setData({ mapScale: 1, mapOffsetX: 1, mapOffsetY: 1 }, () => {
      this.setData({ mapOffsetX: 0, mapOffsetY: 0 })
    })
  },

  mapTap(event) {
    if (!this.data.adding || !this.data.mapMeta) return
    if (Date.now() - (this._lastMapMoveAt || 0) < 180) return
    wx.createSelectorQuery().in(this).select('.map-content').boundingClientRect(rect => {
      if (rect) this._addWaypointAtTouch(event, rect)
    }).exec()
  },

  _addWaypointAtTouch(event, contentRect) {
    if (!this.data.adding || !this.data.mapMeta || !this._stageRect) return
    const touch = (event.touches && event.touches[0]) || event.changedTouches && event.changedTouches[0]
    const clientX = touch
      ? (touch.clientX !== undefined ? touch.clientX : touch.pageX)
      : event.detail && event.detail.x
    const clientY = touch
      ? (touch.clientY !== undefined ? touch.clientY : touch.pageY)
      : event.detail && event.detail.y
    if (!Number.isFinite(clientX) || !Number.isFinite(clientY)) return
    const scale = this._mapScale || this.data.mapScale || 1
    const px = (clientX - contentRect.left) / scale
    const py = (clientY - contentRect.top) / scale
    const rect = this.renderRect()
    if (px < rect.left || px > rect.left + rect.width || py < rect.top || py > rect.top + rect.height) return
    const imageX = (px - rect.left) / rect.scale
    const imageY = (py - rect.top) / rect.scale
    const meta = this.data.mapMeta
    const localX = imageX * meta.resolution
    const localY = (meta.height - imageY) * meta.resolution
    const originYaw = Number(meta.origin_yaw || 0)
    const cosYaw = Math.cos(originYaw)
    const sinYaw = Math.sin(originYaw)
    const point = {
      id: Date.now(),
      name: `目标点 ${this.data.waypoints.length + 1}`,
      x: meta.origin_x + localX * cosYaw - localY * sinYaw,
      y: meta.origin_y + localX * sinYaw + localY * cosYaw,
      yaw: 0
    }
    this.setData({ waypoints: this.data.waypoints.concat([point]), adding: false }, () => this.drawOverlay())
  },

  renderRect() {
    const meta = this.data.mapMeta
    const scale = Math.min(this.data.canvasWidth / meta.width, this.data.canvasHeight / meta.height)
    const width = meta.width * scale
    const height = meta.height * scale
    return { scale, width, height, left: (this.data.canvasWidth - width) / 2, top: (this.data.canvasHeight - height) / 2 }
  },

  worldToCanvas(x, y) {
    const meta = this.data.mapMeta
    if (!meta) return null
    const rect = this.renderRect()
    const dx = x - meta.origin_x
    const dy = y - meta.origin_y
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

  drawOverlay() {
    if (!this.data.mapMeta) return
    const ctx = wx.createCanvasContext('indoorOverlay', this)
    ctx.clearRect(0, 0, this.data.canvasWidth, this.data.canvasHeight)
    const drawPoint = (point, color, label) => {
      const p = this.worldToCanvas(point.x, point.y)
      if (!p) return
      ctx.setFillStyle(color); ctx.beginPath(); ctx.arc(p.x, p.y, 10, 0, Math.PI * 2); ctx.fill()
      ctx.setFillStyle('#0f172a'); ctx.setFontSize(12); ctx.fillText(label, p.x + 12, p.y + 4)
    }
    const path = this.data.pathPoints || []
    if (path.length > 1) {
      ctx.setStrokeStyle('#22c55e'); ctx.setLineWidth(4); ctx.beginPath()
      path.forEach((point, index) => { const p = this.worldToCanvas(point.x, point.y); if (!p) return; index ? ctx.lineTo(p.x, p.y) : ctx.moveTo(p.x, p.y) })
      ctx.stroke()
    }
    this.data.waypoints.forEach((point, index) => drawPoint(point, '#f59e0b', `${index + 1}`))
    const robot = this.worldToCanvas(this.data.state.x, this.data.state.y)
    if (robot && this.data.state.online) {
      const yaw = Number(this.data.state.yaw || 0) * Math.PI / 180
      const screenYaw = yaw - Number(this.data.mapMeta.origin_yaw || 0)
      ctx.setFillStyle('#2563eb'); ctx.beginPath(); ctx.arc(robot.x, robot.y, 12, 0, Math.PI * 2); ctx.fill()
      ctx.setStrokeStyle('#1e3a8a'); ctx.setLineWidth(5); ctx.beginPath(); ctx.moveTo(robot.x, robot.y); ctx.lineTo(robot.x + Math.cos(-screenYaw) * 28, robot.y + Math.sin(-screenYaw) * 28); ctx.stroke()
    }
    ctx.draw()
  },

  moveUp(event) { this.reorder(Number(event.currentTarget.dataset.index), -1) },
  moveDown(event) { this.reorder(Number(event.currentTarget.dataset.index), 1) },
  reorder(index, offset) {
    const target = index + offset
    if (target < 0 || target >= this.data.waypoints.length) return
    const points = this.data.waypoints.slice(); const item = points.splice(index, 1)[0]; points.splice(target, 0, item)
    this.setData({ waypoints: points }, () => this.drawOverlay())
  },
  removePoint(event) {
    const points = this.data.waypoints.slice(); points.splice(Number(event.currentTarget.dataset.index), 1)
    this.setData({ waypoints: points }, () => this.drawOverlay())
  },
  clearPoints() { this.setData({ waypoints: [] }, () => this.drawOverlay()) },
  setPatrolMode(event) { this.setData({ patrolMode: event.currentTarget.dataset.mode }) },
  dwellChange(event) { this.setData({ dwellSeconds: Number(event.detail.value) }) },
  speedChange(event) {
    const speed = Number(event.detail.value)
    this.setData({ speed })
    wx.showToast({ title: `速度设置为 ${speed}%（任务参数）`, icon: 'none' })
  },
  recordPoint() {
    if (!this.data.state.online) return wx.showToast({ title: '机器人离线', icon: 'none' })
    app.globalData.api.sendCommand('INDOOR_RECORD_POINT').catch(error => wx.showToast({ title: error.message, icon: 'none' }))
  },
  startMission() {
    if (!this.data.state.online) return wx.showToast({ title: '机器人离线', icon: 'none' })
    if (!this.data.waypoints.length) return wx.showToast({ title: '请先添加目标点', icon: 'none' })
    app.globalData.api.sendCommand('INDOOR_MISSION_START', {
      mission_id: `mission_${Date.now()}`,
      patrol_mode: 'ONCE',
      dwell_seconds: this.data.dwellSeconds,
      speed: this.data.speed,
      waypoints: this.data.waypoints
    }).then(() => wx.showToast({ title: '导航任务已发送', icon: 'success' })).catch(error => wx.showToast({ title: error.message, icon: 'none' }))
  },
  cancelMission() { app.globalData.api.sendCommand('INDOOR_MISSION_CANCEL').catch(error => wx.showToast({ title: error.message, icon: 'none' })) },
  emergency() {
    wx.showModal({ title: '确认急停', content: '急停后需要人工确认才能恢复运动。', confirmColor: '#dc2626', success: result => {
      if (result.confirm) app.globalData.api.sendCommand('EMERGENCY').then(() => wx.showToast({ title: '急停已发送', icon: 'none' })).catch(error => wx.showToast({ title: error.message, icon: 'none' }))
    } })
  }
})
