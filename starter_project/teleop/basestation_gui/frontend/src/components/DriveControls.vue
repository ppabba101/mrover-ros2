<template>

    <!-- TODO: add HTML to display variables -->

</template>
  
<script>
import { mapActions } from 'vuex'
import { mapState } from 'vuex'

const UPDATE_HZ = 20

export default {
data() {
    return {
    // TODO: add variables to this component
    }
},

watch: {
    // TODO: add watcher for websocket messages
},

methods: {
    ...mapActions('websocket', ['sendMessage'])
},

computed: {
    ...mapState('websocket', ['message'])
},

beforeUnmount: function() {
    window.clearInterval(this.interval)
},

created: function() {
    this.interval = window.setInterval(() => {
    /* 
        To test this code either plug in the Thrustmaster and use this code:
    */

    const gamepads = navigator.getGamepads()
    const gamepad = gamepads.find(gamepad => gamepad && gamepad.id.includes('Thrustmaster'))
    if (!gamepad) return

    // TODO: Send message through websocket here

    
    }, 1000 / UPDATE_HZ)
}
}
</script>

<style scoped>
</style>