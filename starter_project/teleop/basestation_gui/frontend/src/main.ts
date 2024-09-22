import { createApp } from 'vue'
import App from './App.vue'
import router from './router'
import { store } from './store'

import './app.scss'

const app = createApp(App)
app.use(router).use(store).mount('#app')
store.dispatch('websocket/setupWebSocket')
