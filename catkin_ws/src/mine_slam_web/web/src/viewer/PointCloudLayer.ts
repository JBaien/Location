import * as THREE from 'three';
import { ColorMode, colorForPoint } from './ColorMap';

import type { ParsedCloud } from './CloudTypes';
export type { ParsedCloud } from './CloudTypes';

export interface LayerDebug {
  layer_id: string;
  source_topic: string;
  point_count: number;
  visible: boolean;
  geometry_position_count: number;
  bbox_min: [number, number, number] | null;
  bbox_max: [number, number, number] | null;
  last_packet_stamp: string;
  last_update_reason: string;
  packet_type: string;
  replace_or_append: 'replace';
}

export class PointCloudLayer {
  readonly points: THREE.Points;
  private geometry = new THREE.BufferGeometry();
  private material = new THREE.PointsMaterial({ size: 0.035, vertexColors: true, sizeAttenuation: true });
  private cloud: ParsedCloud | null = null;
  private colors = new Float32Array(0);
  private visible = true;
  private debug: LayerDebug;

  constructor(private readonly layerId = 'cloud_layer', private sourceTopic = 'unknown') {
    this.points = new THREE.Points(this.geometry, this.material);
    this.points.frustumCulled = false;
    this.points.renderOrder = 2;
    this.debug = {
      layer_id: this.layerId,
      source_topic: this.sourceTopic,
      point_count: 0,
      visible: true,
      geometry_position_count: 0,
      bbox_min: null,
      bbox_max: null,
      last_packet_stamp: '0',
      last_update_reason: 'init',
      packet_type: 'none',
      replace_or_append: 'replace'
    };
  }

  setVisible(visible: boolean): void {
    this.visible = visible;
    this.points.visible = visible;
    this.debug.visible = visible;
  }

  setSourceTopic(sourceTopic: string): void {
    this.sourceTopic = sourceTopic || 'unknown';
    this.debug.source_topic = this.sourceTopic;
  }

  setPointSize(size: number): void {
    this.material.size = THREE.MathUtils.clamp(size, 0.005, 0.2);
    this.material.needsUpdate = true;
  }

  updateCloud(cloud: ParsedCloud, mode: ColorMode, reflectorVisible: boolean, layerVisible = this.visible): void {
    this.cloud = cloud;
    this.colors = new Float32Array(cloud.pointCount * 3);
    if (cloud.pointCount === 0) {
      this.geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(0), 3));
      this.geometry.setAttribute('color', new THREE.BufferAttribute(new Float32Array(0), 3));
    } else {
      this.geometry.setAttribute('position', new THREE.BufferAttribute(cloud.positions, 3));
      this.geometry.setAttribute('color', new THREE.BufferAttribute(this.colors, 3));
      this.recolor(mode, reflectorVisible);
    }
    const positionAttribute = this.geometry.getAttribute('position');
    const colorAttribute = this.geometry.getAttribute('color');
    if (positionAttribute) positionAttribute.needsUpdate = true;
    if (colorAttribute) colorAttribute.needsUpdate = true;
    this.geometry.computeBoundingBox();
    this.geometry.computeBoundingSphere();
    this.points.frustumCulled = false;
    this.setVisible(layerVisible);
    this.updateDebug(cloud, 'packet_replace');
  }

  recolor(mode: ColorMode, reflectorVisible: boolean): void {
    if (!this.cloud) return;
    const positions = this.cloud.positions;
    for (let i = 0; i < this.cloud.pointCount; i += 1) {
      const [r, g, b] = colorForPoint(
        {
          x: positions[i * 3],
          y: positions[i * 3 + 1],
          z: positions[i * 3 + 2],
          intensity: this.cloud.intensities[i],
          lidarId: this.cloud.lidarIds[i],
          classId: this.cloud.classIds[i],
          cloudType: this.cloud.cloudType
        },
        mode,
        reflectorVisible
      );
      this.colors[i * 3] = r;
      this.colors[i * 3 + 1] = g;
      this.colors[i * 3 + 2] = b;
    }
    const colorAttribute = this.geometry.getAttribute('color');
    if (colorAttribute) {
      colorAttribute.needsUpdate = true;
    }
  }

  getBoundingBox(): THREE.Box3 | null {
    this.geometry.computeBoundingBox();
    return this.geometry.boundingBox ? this.geometry.boundingBox.clone() : null;
  }

  getDebug(): LayerDebug {
    this.updateDebug(this.cloud, this.debug.last_update_reason);
    return { ...this.debug };
  }

  private updateDebug(cloud: ParsedCloud | null, reason: string): void {
    const positionAttribute = this.geometry.getAttribute('position') as THREE.BufferAttribute | undefined;
    const bbox = this.geometry.boundingBox;
    this.debug = {
      layer_id: this.layerId,
      source_topic: this.sourceTopic,
      point_count: cloud?.pointCount ?? 0,
      visible: this.points.visible,
      geometry_position_count: positionAttribute?.count ?? 0,
      bbox_min: bbox ? [bbox.min.x, bbox.min.y, bbox.min.z] : null,
      bbox_max: bbox ? [bbox.max.x, bbox.max.y, bbox.max.z] : null,
      last_packet_stamp: cloud?.stampNs.toString() ?? '0',
      last_update_reason: reason,
      packet_type: cloud ? `cloud_type_${cloud.cloudType}` : 'none',
      replace_or_append: 'replace'
    };
  }
}
