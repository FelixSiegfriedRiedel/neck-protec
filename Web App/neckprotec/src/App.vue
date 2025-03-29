<script>
import { defineComponent } from 'vue';
import { useWebNotification } from '@vueuse/core';
import { createToast } from 'mosha-vue-toastify';
import 'mosha-vue-toastify/dist/style.css';



export default {

  data() {
    return {
      deviceConnected: false,
      deviceName: '',
      accAngles: [],
      isPoorPosture: 0,
      device: null,
      imgName: "/neck_angles/neck_angle%20(1).jpg",
      translatedAngle: 0,
      lastTriggerTime: 0
    };
  },

  setup() {
    const options = {
      title: 'Achtung! Sie haben eine ungesunde Nackenhaltung.',
      dir: 'auto',
      lang: 'de',
      renotify: true,
      tag: 'Nackenhaltung',
    };

    function ringBell(){
      const sound = new Audio('sounds/notification.mp3');
      sound.play();
    }

    const { isSupported, show } = useWebNotification(options);

    const toast = () => {
      createToast('Achtung! Sie haben eine ungesunde Nackenhaltung.');
    };
    return { isSupported, show, toast, ringBell};
  },

  methods: {
    async connectDevice() {
      console.log("Connecting..");
      try {
        const device = await navigator.bluetooth.requestDevice({
          filters: [{ services: ['0176b37a-1ede-4268-baae-ac5a3b1b2675'] }],
        });

        this.device = device;
        const server = await device.gatt.connect();
        this.deviceConnected = true;
        this.deviceName = device.name;
        console.log("Connected!");


        await this.startNotifications(server);
      } catch (error) {
        console.error('Connection failed:', error);
      }
    },

    async startNotifications(server) {
      try {
        const service = await server.getPrimaryService('0176b37a-1ede-4268-baae-ac5a3b1b2675');

        // Acc Angles Notifications
        const accAnglesCharacteristic = await service.getCharacteristic('d1fb867a-4653-4e62-950e-5f20c725e683');
        await accAnglesCharacteristic.startNotifications();
        accAnglesCharacteristic.addEventListener('characteristicvaluechanged', this.handleAccAnglesUpdate);

        // Neck Position Notifications
        const neckPosCharacteristic = await service.getCharacteristic('df6db541-5894-4eff-8384-087583af5bbf');
        await neckPosCharacteristic.startNotifications();
        neckPosCharacteristic.addEventListener('characteristicvaluechanged', this.handleNeckPosUpdate);
      } catch (error) {
        console.error('Failed to set up notifications:', error);
      }
    },

    handleAccAnglesUpdate(event) {
      const value = new TextDecoder().decode(event.target.value);
      this.accAngles = value.split('|');
      this.getImageName(this.accAngles[1]);
    },

    handleNeckPosUpdate(event) {
      console.log('New NeckPos Data received:', event);
      this.isPoorPosture = event.target.value.getUint8(0);

      const currentTime = Date.now();  // Get the current time in milliseconds
      const timeDifference = currentTime - this.lastTriggerTime;

      if (this.isPoorPosture === 1 && timeDifference > 10000) {
        // If it's been more than 10 seconds since the last trigger, proceed
        if (this.isSupported) {
          this.show();
        } else {
          this.toast();
          this.ringBell();
        }

        // Update the last trigger time to the current time
        this.lastTriggerTime = currentTime;
      }
    },

    notificationTest() {
      if (!this.isSupported) {
        this.show();
      } else {
        this.toast();
        this.ringBell();
      }
    },

    getImageName(angle) {
      function mapRange(value, fromMin, fromMax, toMin, toMax) {
        return toMin + (value - fromMin) * (toMax - toMin) / (fromMax - fromMin);
      }

      let mappedValue = 1;
      if (angle < -90) {
        mappedValue = 1;
      } else if (angle > -30) {
        mappedValue = 40;
      } else {
        mappedValue = Math.round(mapRange(angle, -90, -30, 1, 40));
      }

      this.translatedAngle = Math.round(angle)
      this.translatedAngle += 90
      this.imgName = `/neck_angles/neck_angle%20(${mappedValue}).jpg`;
      // console.log(this.imgName);
      // console.log(this.translatedAngle);
    },
  },
};
</script>


<template>
  <div id="app" class="container">
    <div>
      <h1><b>NeckProtec</b></h1>
      <p><i>Dein smarter Nacken-Sensor.</i></p>
      <button v-if="!deviceConnected" @click="connectDevice">Gerät verbinden</button>
    </div>

    <button  @click="notificationTest">Test</button>

    <div v-if="deviceConnected" class="neck-angle-display">
      <img :src="imgName" alt="" />
      <p class="angle-text">  Nackenwinkel: {{this.translatedAngle}}°</p>
    </div>

  </div>
</template>
