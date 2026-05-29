<script setup lang="ts">
import type { ColorMode } from '../viewer/ColorMap';

defineProps<{ mode: ColorMode }>();
const emit = defineEmits<{ change: [mode: ColorMode] }>();

const modes: Array<{ value: ColorMode; label: string }> = [
  { value: 'height', label: 'Height' },
  { value: 'intensity', label: 'Intensity' },
  { value: 'x-distance', label: 'X distance' },
  { value: 'y-distance', label: 'Y distance' },
  { value: 'z-distance', label: 'Z distance' },
  { value: 'lidar_id', label: 'LiDAR ID' },
  { value: 'stable-temporary', label: 'Stable / temporary' },
  { value: 'reflector', label: 'Reflector' }
];
</script>

<template>
  <section class="panel-section">
    <h2>Color</h2>
    <select class="select" :value="mode" @change="emit('change', ($event.target as HTMLSelectElement).value as ColorMode)">
      <option v-for="item in modes" :key="item.value" :value="item.value">{{ item.label }}</option>
    </select>
  </section>
</template>
