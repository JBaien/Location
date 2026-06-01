import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { ColorMode } from './ColorMap';
import { LayerDebug, ParsedCloud, PointCloudLayer } from './PointCloudLayer';
import { PathLayer } from './PathLayer';

export interface LayerState {
  current: boolean;
  stable: boolean;
  path: boolean;
  reflector: boolean;
  grid: boolean;
}

export interface RenderStats {
  fps: number;
  frameMs: number;
}

export interface SceneDebug {
  current: LayerDebug;
  stable: LayerDebug;
  path_position_count: number;
}

export class SceneView {
  readonly currentLayer = new PointCloudLayer('current_cloud');
  readonly stableLayer = new PointCloudLayer('stable_map');
  readonly pathLayer = new PathLayer();

  private scene = new THREE.Scene();
  private camera = new THREE.PerspectiveCamera(60, 1, 0.05, 5000);
  private renderer = new THREE.WebGLRenderer({ antialias: true });
  private controls: OrbitControls;
  private grid: THREE.GridHelper;
  private axisGuide = new THREE.Group();
  private resizeObserver: ResizeObserver;
  private animationFrame = 0;
  private colorMode: ColorMode = 'height';
  private layerState: LayerState = { current: true, stable: true, path: true, reflector: true, grid: true };
  private statsCallback: ((stats: RenderStats) => void) | null = null;
  private lastFrameTime = performance.now();
  private statsWindowStart = performance.now();
  private statsFrames = 0;
  private autoFitStableDone = false;
  private autoFitCurrentDone = false;

  constructor(private readonly host: HTMLElement) {
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.setClearColor(0x2f3336, 1);
    this.host.appendChild(this.renderer.domElement);

    this.camera.up.set(0, 0, 1);
    this.camera.position.set(-8, -12, 7);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(10, 0, 0);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.06;
    this.controls.enablePan = true;
    this.controls.enableRotate = true;
    this.controls.enableZoom = true;
    this.controls.screenSpacePanning = true;
    this.controls.minDistance = 0.05;
    this.controls.maxDistance = 10000;
    this.controls.minPolarAngle = 0;
    this.controls.maxPolarAngle = Math.PI;
    this.controls.rotateSpeed = 0.82;
    this.controls.panSpeed = 1.0;
    this.controls.zoomSpeed = 1.12;
    this.controls.mouseButtons = {
      LEFT: THREE.MOUSE.ROTATE,
      MIDDLE: THREE.MOUSE.DOLLY,
      RIGHT: THREE.MOUSE.PAN
    };
    this.controls.touches = {
      ONE: THREE.TOUCH.ROTATE,
      TWO: THREE.TOUCH.DOLLY_PAN
    };
    if ('zoomToCursor' in this.controls) {
      this.controls.zoomToCursor = true;
    }

    this.grid = new THREE.GridHelper(120, 60, 0x6f777d, 0x454b50);
    this.grid.rotation.x = Math.PI / 2;
    this.scene.add(this.grid);
    this.axisGuide = this.createAxisGuide();
    this.scene.add(this.axisGuide);
    this.scene.add(this.currentLayer.points);
    this.scene.add(this.stableLayer.points);
    this.scene.add(this.pathLayer.line);
    this.scene.add(this.pathLayer.poseMarker);

    this.resizeObserver = new ResizeObserver(() => this.resize());
    this.resizeObserver.observe(this.host);
    this.resize();
    this.render();
  }

  dispose(): void {
    window.cancelAnimationFrame(this.animationFrame);
    this.resizeObserver.disconnect();
    this.renderer.dispose();
    this.host.innerHTML = '';
  }

  updateCloud(cloud: ParsedCloud): void {
    if (cloud.cloudType === 1) {
      this.currentLayer.updateCloud(cloud, this.colorMode, this.layerState.reflector, this.layerState.current);
      if (!this.autoFitCurrentDone && !this.autoFitStableDone && cloud.pointCount > 0) {
        this.fitCurrent();
        this.autoFitCurrentDone = true;
      }
    } else if (cloud.cloudType === 2) {
      this.stableLayer.updateCloud(cloud, this.colorMode, this.layerState.reflector, this.layerState.stable);
      if (!this.autoFitStableDone && cloud.pointCount > 0) {
        this.fitStable();
        this.autoFitStableDone = true;
      }
    }
  }

  setSourceTopics(currentTopic?: string, stableTopic?: string): void {
    if (currentTopic) this.currentLayer.setSourceTopic(currentTopic);
    if (stableTopic) this.stableLayer.setSourceTopic(stableTopic);
  }

  updatePath(path: number[][], pose?: { valid: boolean; x: number; y: number; z: number }): void {
    this.pathLayer.update(path, pose);
  }

  setLayers(layers: LayerState): void {
    this.layerState = { ...layers };
    this.currentLayer.setVisible(layers.current);
    this.stableLayer.setVisible(layers.stable);
    this.pathLayer.setVisible(layers.path);
    this.grid.visible = layers.grid;
    this.recolor();
  }

  setColorMode(mode: ColorMode): void {
    this.colorMode = mode;
    this.recolor();
  }

  setPointSize(size: number): void {
    this.currentLayer.setPointSize(size);
    this.stableLayer.setPointSize(size);
  }

  onStats(callback: (stats: RenderStats) => void): void {
    this.statsCallback = callback;
  }

  fitCurrent(): void {
    this.fitBox(this.currentLayer.getBoundingBox());
  }

  fitStable(): void {
    this.fitBox(this.stableLayer.getBoundingBox());
  }

  fitAll(): void {
    const box = new THREE.Box3();
    let hasBox = false;
    for (const next of [this.currentLayer.getBoundingBox(), this.stableLayer.getBoundingBox(), this.pathLayer.getBoundingBox()]) {
      if (!next || next.isEmpty()) continue;
      if (!hasBox) {
        box.copy(next);
        hasBox = true;
      } else {
        box.union(next);
      }
    }
    this.fitBox(hasBox ? box : null);
  }

  viewIso(): void {
    this.setCameraView(new THREE.Vector3(-0.65, -0.85, 0.48));
  }

  viewTop(): void {
    this.setCameraView(new THREE.Vector3(0, 0, 1));
  }

  viewSide(): void {
    this.setCameraView(new THREE.Vector3(0, -1, 0.12));
  }

  viewEnd(): void {
    this.setCameraView(new THREE.Vector3(-1, 0, 0.12));
  }

  getDebug(): SceneDebug {
    return {
      current: this.currentLayer.getDebug(),
      stable: this.stableLayer.getDebug(),
      path_position_count: this.pathLayer.getPointCount()
    };
  }

  private recolor(): void {
    this.currentLayer.recolor(this.colorMode, this.layerState.reflector);
    this.stableLayer.recolor(this.colorMode, this.layerState.reflector);
  }

  private createAxisGuide(): THREE.Group {
    const group = new THREE.Group();
    group.name = 'xy_axis_guide';

    const origin = new THREE.Vector3(0, 0, 0.03);
    const xColor = 0xff5c5c;
    const yColor = 0x42d67b;
    const arrowLength = 0.72;
    const shaftRadius = 0.018;
    const headLength = 0.16;
    const headRadius = 0.06;

    group.add(this.createThickArrow(new THREE.Vector3(1, 0, 0), origin, arrowLength, xColor, shaftRadius, headLength, headRadius));
    group.add(this.createThickArrow(new THREE.Vector3(0, 1, 0), origin, arrowLength, yColor, shaftRadius, headLength, headRadius));

    const xLabel = this.createAxisLabel('+X', '#ff8b8b');
    xLabel.position.set(arrowLength + 0.2, 0, 0.08);
    group.add(xLabel);

    const yLabel = this.createAxisLabel('+Y', '#70e99d');
    yLabel.position.set(0, arrowLength + 0.2, 0.08);
    group.add(yLabel);

    return group;
  }

  private createAxisLabel(text: string, color: string): THREE.Sprite {
    const canvas = document.createElement('canvas');
    canvas.width = 256;
    canvas.height = 96;
    const ctx = canvas.getContext('2d');
    if (ctx) {
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      ctx.font = '800 58px "Segoe UI", "Noto Sans SC", sans-serif';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.lineWidth = 7;
      ctx.strokeStyle = 'rgba(5, 12, 18, 0.92)';
      ctx.strokeText(text, canvas.width / 2, canvas.height / 2);
      ctx.fillStyle = color;
      ctx.fillText(text, canvas.width / 2, canvas.height / 2);
    }

    const texture = new THREE.CanvasTexture(canvas);
    texture.colorSpace = THREE.SRGBColorSpace;
    const material = new THREE.SpriteMaterial({
      map: texture,
      transparent: true,
      depthTest: false,
      depthWrite: false
    });
    const sprite = new THREE.Sprite(material);
    sprite.scale.set(0.64, 0.28, 1);
    sprite.renderOrder = 10;
    return sprite;
  }

  private createThickArrow(
    direction: THREE.Vector3,
    origin: THREE.Vector3,
    length: number,
    color: number,
    shaftRadius: number,
    headLength: number,
    headRadius: number
  ): THREE.Group {
    const group = new THREE.Group();
    const dir = direction.clone().normalize();
    const shaftLength = Math.max(0.01, length - headLength);
    const material = new THREE.MeshBasicMaterial({ color });

    const shaft = new THREE.Mesh(
      new THREE.CylinderGeometry(shaftRadius, shaftRadius, shaftLength, 16),
      material
    );
    shaft.position.copy(origin.clone().addScaledVector(dir, shaftLength / 2));

    const head = new THREE.Mesh(
      new THREE.ConeGeometry(headRadius, headLength, 24),
      material
    );
    head.position.copy(origin.clone().addScaledVector(dir, shaftLength + headLength / 2));

    const quat = new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir);
    shaft.quaternion.copy(quat);
    head.quaternion.copy(quat);
    group.add(shaft);
    group.add(head);
    return group;
  }

  private resize(): void {
    const width = Math.max(1, this.host.clientWidth);
    const height = Math.max(1, this.host.clientHeight);
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(width, height, false);
  }

  private fitBox(box: THREE.Box3 | null): void {
    if (!box || box.isEmpty()) return;
    const center = box.getCenter(new THREE.Vector3());
    const size = box.getSize(new THREE.Vector3());
    const radius = Math.max(size.x, size.y, size.z, 1);
    this.applyCamera(center, new THREE.Vector3(-0.55, -0.9, 0.45), radius);
  }

  private setCameraView(direction: THREE.Vector3): void {
    const box = new THREE.Box3();
    let hasBox = false;
    for (const next of [this.stableLayer.getBoundingBox(), this.currentLayer.getBoundingBox(), this.pathLayer.getBoundingBox()]) {
      if (!next || next.isEmpty()) continue;
      if (!hasBox) {
        box.copy(next);
        hasBox = true;
      } else {
        box.union(next);
      }
    }
    if (!hasBox) return;
    const center = box.getCenter(new THREE.Vector3());
    const size = box.getSize(new THREE.Vector3());
    const radius = Math.max(size.x, size.y, size.z, 1);
    this.applyCamera(center, direction, radius);
  }

  private applyCamera(center: THREE.Vector3, direction: THREE.Vector3, radius: number): void {
    const viewDirection = direction.clone().normalize();
    const distance = Math.max(radius * 1.45, 4);
    this.controls.target.copy(center);
    this.camera.up.set(0, 0, 1);
    if (Math.abs(viewDirection.z) > 0.95) {
      this.camera.up.set(0, 1, 0);
    }
    this.camera.position.copy(center.clone().addScaledVector(viewDirection, distance));
    this.camera.near = Math.max(0.01, radius / 10000);
    this.camera.far = Math.max(5000, radius * 20);
    this.camera.updateProjectionMatrix();
    this.controls.update();
  }

  private render = (): void => {
    const now = performance.now();
    const frameMs = now - this.lastFrameTime;
    this.lastFrameTime = now;
    this.statsFrames += 1;
    const windowMs = now - this.statsWindowStart;
    if (this.statsCallback && windowMs >= 1000) {
      this.statsCallback({ fps: (this.statsFrames * 1000) / windowMs, frameMs });
      this.statsFrames = 0;
      this.statsWindowStart = now;
    }
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
    this.animationFrame = window.requestAnimationFrame(this.render);
  };
}
