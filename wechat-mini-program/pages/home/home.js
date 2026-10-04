const app = getApp()
const modes = { 1: 'GPS+ROS / 融合导航', 2: 'REMOTE / 遥控', 3: 'INDOOR / 室内导航', 4: 'GPS ONLY / 纯GPS' }
const modeOptions = [
  { value: 2, label: '遥控模式', command: 'REMOTE', url: '/pages/control/control' },
  { value: 3, label: '室内导航', command: 'INDOOR', url: '/pages/indoor/indoor' }
]
Page({
  data: { state: app.globalData.state, modeName: '', lastSeen: '', modeOptions, switchingMode: 0 },
  onLoad() { this.unsubscribe = app.globalData.api.onState(state => this.applyState(state)) },
  onUnload() { if (this.unsubscribe) this.unsubscribe() },
  onShow() { this.refresh() },
  applyState(state) {
    const lastSeen = state.last_message_at ? `${Math.max(0, Math.floor((Date.now() - state.last_message_at) / 1000))} 秒前` : '尚未收到'
    const confirmed = this.data.switchingMode && state.mode === this.data.switchingMode
    this.setData({
      state,
      modeName: modes[state.mode] || '未知模式',
      lastSeen,
      switchingMode: confirmed ? 0 : this.data.switchingMode
    }, () => {
      if (confirmed && this._pendingModeUrl) {
        const url = this._pendingModeUrl
        this._pendingModeUrl = ''
        wx.switchTab({ url })
      }
    })
  },
  refresh() {
    this.applyState(app.globalData.api.getState())
  },
  reconnect() {
    app.globalData.api.reconnect()
    wx.showToast({ title: '正在重新连接', icon: 'none' })
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
