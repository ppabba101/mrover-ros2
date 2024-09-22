import websocket from './modules/websocket'
import { createStore } from 'vuex'

export const store = createStore({
  modules: {
    websocket
  }
})