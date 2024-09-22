import { createWebHistory, createRouter } from "vue-router";
import Menu from "../components/MenuPage.vue";

// TODO: Import DriveControls here

const routes = [
  {
    path: "/",
    name: "Menu",
    component: Menu,
  },
  // TODO: Add DriveControls route here
];

const router = createRouter({
  history: createWebHistory(),
  routes,
});

export default router;