<template>
  <div class="btn btn-primary">
    <a :href="link">{{ name }}</a>
  </div>
</template>

<script>
import { defineComponent } from 'vue'

export default defineComponent({
  name: 'MenuButton',

  props: {
    link: {
      type: String,
      required: true
    },
    name: {
      type: String,
      required: true
    }
  }
})
</script>

<style scoped>
a {
  margin: 5px;
  width: 250px;
  height: 150px;

  text-decoration: none;
  color: white;
}
</style>
