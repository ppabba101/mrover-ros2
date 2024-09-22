<template>
    <div class="wrapper">
      <div class="shadow p-3 mb-5 header">
        <h1>Menu</h1>
        <img class="logo" src="/mrover.png" alt="MRover" title="MRover" width="200" />
      </div>
      <div>
        <MenuButton link="/motor_sim" name="Motor Simulator"></MenuButton>
      </div>
    </div>
  </template>
  
  <script>
  import MenuButton from './MenuButton.vue';
  
  export default {
    name: 'MenuPage',
  
  components: {
      MenuButton,
  }
  
  }
  </script>
  
  <style scoped>
  .header {
    grid-area: header;
    display: flex;
    justify-content: space-between;
    align-items: center;
    padding: 10px;
  }
  
  .logo {
    position: absolute;
    left: 50%;
    transform: translateX(-50%);
  }
  
  img {
    border: none;
    border-radius: 0px;
  }
  
  .pages {
    grid-area: pages;
  }
  
  .wrapper {
    display: grid;
    grid-gap: 10px;
    grid-template-columns: 1fr;
    grid-template-rows: auto 1fr;
    grid-template-areas: 'header' 'pages';
    font-family: sans-serif;
    height: auto;
  }
  
  </style>