const api = require('./utils/api')

App({
  globalData: {
    robotId: 'robot_001',
    api,
    state: api.getState()
  },
  onLaunch() {
    api.onState(state => { this.globalData.state = state })
    api.connect()
  }
})
