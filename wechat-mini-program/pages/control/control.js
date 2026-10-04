const app = getApp()
const modes = [{ value: 2, label: '遥控' }, { value: 3, label: '室内导航' }]
const names = { 2: 'REMOTE / 遥控', 3: 'INDOOR / 室内导航' }
Page({
  data: {
    state: app.globalData.state,
    modes,
    modeName: '',
    modeStatus: '',
    selectedMode: 2,
    pendingMode: 0,
    speed: 50,
    mapImagePath: '',
    mapMeta: null,
    mapPathPoints: [],
    mapCanvasWidth: 300,
    mapCanvasHeight: 220
  },
  onLoad() {
    const api = app.globalData.api
    this.unsubscribe = api.onState(state => this.applyState(state))
    this.unsubscribeMap = api.onMap(payload => this.onMap(payload))
    this.unsubscribePath = api.onPath(payload => {
      this.setData({ mapPathPoints: payload.points || [] }, () => this.drawMap())
    })
  },
  onReady() {
    wx.createSelectorQuery().in(this).select('.control-map-stage').boundingClientRect(rect => {
      if (!rect) return
      this.setData({
        mapCanvasWidth: Math.round(rect.width),
        mapCanvasHeight: Math.round(rect.height)
      }, () => this.drawMap())
    }).exec()
  },
  onUnload() {
    this._stopMoveHeartbeat(true)
    if (this.unsubscribe) this.unsubscribe()
    if (this.unsubscribeMap) this.unsubscribeMap()
    if (this.unsubscribePath) this.unsubscribePath()
  },
  onHide() {
    this._stopMoveHeartbeat(true)
  },
  onShow() {
    const state = app.globalData.api.getState()
    if (!state.online || state.mode !== 2) {
      wx.showToast({ title: '请先切换到遥控模式', icon: 'none' })
      wx.switchTab({ url: '/pages/home/home' })
      return
    }
    this.sync()
    this.drawMap()
  },
  applyState(state) {
    const confirmed = this.data.pendingMode && state.mode === this.data.pendingMode
    const pendingMode = confirmed ? 0 : this.data.pendingMode
    const selectedMode = pendingMode ? this.data.selectedMode : state.mode
    const modeStatus = pendingMode
      ? `切换中，等待机器人确认：${names[pendingMode]}`
      : (state.online ? '机器人在线，模式已确认' : '机器人离线，控制已禁用')
    this.setData({ state, pendingMode, selectedMode, modeName: names[selectedMode] || '未知模式', modeStatus }, () => {
      this.drawMap()
      if (confirmed && this._pendingModeUrl) {
        const url = this._pendingModeUrl
        this._pendingModeUrl = ''
        wx.switchTab({ url })
      }
    })
  },
  sync() { this.applyState(app.globalData.api.getState()) },
  onMap(payload) {
    if (!payload || !payload.png_base64) return
    const path = wx.env.USER_DATA_PATH + '/control_map_' + payload.revision + '.png'
    const previousPath = this.data.mapImagePath
    wx.getFileSystemManager().writeFile({
      filePath: path,
      data: payload.png_base64,
      encoding: 'base64',
      success: () => {
        this.setData({ mapImagePath: path, mapMeta: payload }, () => {
          this.drawMap()
          if (previousPath && previousPath !== path) {
            wx.getFileSystemManager().unlink({ filePath: previousPath, fail: () => {} })
          }
        })
      },
      fail: error => console.error('控制页地图保存失败', error)
    })
  },
  mapRenderRect() {
    const meta = this.data.mapMeta
    const scale = Math.min(this.data.mapCanvasWidth / meta.width, this.data.mapCanvasHeight / meta.height)
    const width = meta.width * scale
    const height = meta.height * scale
    return {
      scale,
      width,
      height,
      left: (this.data.mapCanvasWidth - width) / 2,
      top: (this.data.mapCanvasHeight - height) / 2
    }
  },
  mapWorldToCanvas(x, y) {
    const meta = this.data.mapMeta
    if (!meta) return null
    const rect = this.mapRenderRect()
    const dx = Number(x || 0) - meta.origin_x
    const dy = Number(y || 0) - meta.origin_y
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
  drawMap() {
    if (!this.data.mapMeta) return
    const ctx = wx.createCanvasContext('controlMapOverlay', this)
    ctx.clearRect(0, 0, this.data.mapCanvasWidth, this.data.mapCanvasHeight)
    const path = this.data.mapPathPoints || []
    if (path.length > 1) {
      ctx.setStrokeStyle('#22c55e')
      ctx.setLineWidth(4)
      ctx.beginPath()
      path.forEach((point, index) => {
        const p = this.mapWorldToCanvas(point.x, point.y)
        if (!p) return
        if (index) ctx.lineTo(p.x, p.y)
        else ctx.moveTo(p.x, p.y)
      })
      ctx.stroke()
    }
    const robot = this.mapWorldToCanvas(this.data.state.x, this.data.state.y)
    if (robot && this.data.state.online) {
      const yaw = Number(this.data.state.yaw || 0) * Math.PI / 180
      const screenYaw = yaw - Number(this.data.mapMeta.origin_yaw || 0)
      ctx.setFillStyle('#2563eb')
      ctx.beginPath()
      ctx.arc(robot.x, robot.y, 10, 0, Math.PI * 2)
      ctx.fill()
      ctx.setStrokeStyle('#172554')
      ctx.setLineWidth(4)
      ctx.beginPath()
      ctx.moveTo(robot.x, robot.y)
      ctx.lineTo(robot.x + Math.cos(-screenYaw) * 24, robot.y + Math.sin(-screenYaw) * 24)
      ctx.stroke()
    }
    ctx.draw()
  },
  goIndoor() {
    this.switchMode({ currentTarget: { dataset: { mode: 3 } } })
  },
  showError(error) { wx.showToast({ title: error.message || '发送失败', icon: 'none' }) },
  switchMode(e) {
    this._stopMoveHeartbeat(true)
    const mode = Number(e.currentTarget.dataset.mode)
    const previousMode = this.data.selectedMode
    const command = { 2: 'REMOTE', 3: 'INDOOR' }[mode]
    if (!this.data.state.online) return wx.showToast({ title: '机器人离线', icon: 'none' })

    this.setData({
      selectedMode: mode,
      pendingMode: mode,
      modeName: names[mode],
      modeStatus: `切换中，正在发送：${names[mode]}`
    })

    this._pendingModeUrl = {
      2: '/pages/control/control',
      3: '/pages/indoor/indoor'
    }[mode]
    app.globalData.api.sendCommand(command).then(() => {
      this.setData({ modeStatus: `指令已发送，等待机器人确认：${names[mode]}` })
      wx.showToast({ title: '切换指令已发送', icon: 'success' })
    }).catch(error => {
      this._pendingModeUrl = ''
      this.setData({ selectedMode: previousMode, pendingMode: 0, modeName: names[previousMode] })
      this.showError(error)
    })
  },
  _sendMoveCommand(command, showError = false) {
    return app.globalData.api.sendCommand(command, { speed: this.data.speed }).catch(error => {
      if (showError) this.showError(error)
      else console.warn('move heartbeat failed', error)
    })
  },
  _stopMoveHeartbeat(sendStop = false) {
    const wasMoving = Boolean(this._activeMoveCommand)
    if (this._moveHeartbeatTimer) {
      clearInterval(this._moveHeartbeatTimer)
      this._moveHeartbeatTimer = null
    }
    this._activeMoveCommand = ''
    if (sendStop && wasMoving) {
      app.globalData.api.sendCommand('STOP', { speed: 0 }).catch(error => console.warn('STOP failed', error))
    }
  },
  move(e) {
    if (!this.data.state.online) return wx.showToast({ title: '机器人离线', icon: 'none' })
    const command = e.currentTarget.dataset.command
    if (!command) return
    this._stopMoveHeartbeat(false)
    this._activeMoveCommand = command
    this._sendMoveCommand(command, true)
    this._moveHeartbeatTimer = setInterval(() => {
      if (!this._activeMoveCommand) return
      if (!this.data.state.online || this.data.state.mode !== 2) {
        this._stopMoveHeartbeat(true)
        return
      }
      this._sendMoveCommand(this._activeMoveCommand)
    }, 200)
  },
  stop() {
    this._stopMoveHeartbeat(false)
    app.globalData.api.sendCommand('STOP', { speed: 0 }).catch(error => this.showError(error))
  },
  emergency() {
    this._stopMoveHeartbeat(true)
    wx.showModal({ title: '确认急停', content: '急停后需要人工确认才能恢复运动。', confirmColor: '#dc2626', success: r => { if (r.confirm) { app.globalData.api.sendCommand('EMERGENCY').then(() => wx.showToast({ title: '急停已发送', icon: 'none' })).catch(error => this.showError(error)) } } })
  },
  speedChange(e) { this.setData({ speed: Number(e.detail.value) }) }
})
