import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const COLORS = [
    [255,80,80],[80,180,255],[80,220,100],[255,200,60],[200,100,255],
    [255,140,60],[60,220,220],[255,100,180],[140,255,140],[100,140,255],
    [220,180,100],[180,100,140],[100,220,180],[220,120,220],[180,220,60],
    [120,160,220],[220,160,160],[160,220,200],[200,140,100],[140,180,160],
    [180,140,220],[220,200,140],[140,220,160],[200,100,180],[160,200,220],
    [220,140,100],[100,180,200],[200,220,120],[160,120,200],[120,200,160],
];

let meta = null;
let layers = [];
let masks = [];
let seamLabels = null;
let originalSeamLabels = null;
let compositeCanvas, compositeCtx, compositeTexture;
let overlayCanvas, overlayCtx, overlayTexture;
let eqWidth = 0, eqHeight = 0;
let texW = 0, texH = 0;
let uncropW = 0, uncropH = 0;
let cropX0 = 0, cropY0 = 0;
let cropPxX = 0, cropPxY = 0;

let selectedImage = -1;
let checkedImages = new Set();
let mode = 'move';
let brushSize = 10;
let dragOpacity = 0.7;
let showOverlay = false;
let draggingHandle = -1;
let dragStartPoint = null;
let dragStartYawPitch = null;
let dragBaseCanvas = null;
let dragLayerCanvas = null;
let dragStartEq = null;
let painting = false;
let layerCanvases = [];
let imageOffsets = [];
let pendingChanges = false;
let dragStartOffset = null;
let lastHitPoint = null;
let moveClickStart = null;
let moveDragging = false;

let scene, camera, renderer, controls;
let sphereMesh, overlayMesh, raycastSphere;
let handleGroup;
let outlineGroup;
let brushCursor;
let raycaster, mouse;

const container = document.getElementById('three-container');
const statusBar = document.getElementById('status-bar');
const loadingOverlay = document.getElementById('loading-overlay');
const loadingText = document.getElementById('loading-text');

function showStatus(msg) {
    statusBar.textContent = msg;
    statusBar.classList.add('visible');
}
function hideStatus() {
    statusBar.classList.remove('visible');
}

function loadImage(url) {
    return new Promise((resolve, reject) => {
        const img = new Image();
        img.onload = () => resolve(img);
        img.onerror = () => reject(new Error(`Failed to load: ${url}`));
        img.src = url;
    });
}

function getImageData(img) {
    const c = document.createElement('canvas');
    c.width = img.width;
    c.height = img.height;
    const cx = c.getContext('2d');
    cx.drawImage(img, 0, 0);
    const data = cx.getImageData(0, 0, c.width, c.height);
    c.width = 0;
    c.height = 0;
    return data;
}

function getMaskData(img) {
    const c = document.createElement('canvas');
    c.width = img.width;
    c.height = img.height;
    const cx = c.getContext('2d');
    cx.drawImage(img, 0, 0);
    const rgba = cx.getImageData(0, 0, c.width, c.height);
    c.width = 0;
    c.height = 0;
    const n = img.width * img.height;
    const mask = new Uint8Array(n);
    for (let i = 0; i < n; i++) mask[i] = rgba.data[i * 4];
    return mask;
}

function yawPitchToSphere(yawDeg, pitchDeg) {
    const lon = yawDeg * Math.PI / 180;
    const lat = pitchDeg * Math.PI / 180;
    return new THREE.Vector3(
        Math.cos(lat) * Math.sin(lon),
        Math.sin(lat),
        Math.cos(lat) * Math.cos(lon)
    );
}

function sphereToYawPitch(point) {
    const lat = Math.asin(Math.max(-1, Math.min(1, point.y)));
    const lon = Math.atan2(point.x, point.z);
    return {
        yaw: lon * 180 / Math.PI,
        pitch: lat * 180 / Math.PI,
    };
}

function yawPitchToEquirect(yawDeg, pitchDeg) {
    const s = eqWidth / uncropW;
    let fullX = (yawDeg / 360 + 0.5) * uncropW;
    const fullY = (0.5 - pitchDeg / 180) * uncropH;
    let x = (fullX - cropX0) * s;
    if (x < 0) x += eqWidth;
    else if (x >= eqWidth) x -= eqWidth;
    return { x, y: (fullY - cropY0) * s };
}

function equirectToYawPitch(ex, ey) {
    const s = eqWidth / uncropW;
    const fullX = ex / s + cropX0;
    const fullY = ey / s + cropY0;
    return {
        yaw: (fullX / uncropW - 0.5) * 360,
        pitch: (0.5 - fullY / uncropH) * 180,
    };
}

function initThree() {
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x1a1a1a);

    const aspect = container.clientWidth / container.clientHeight;
    camera = new THREE.PerspectiveCamera(60, aspect, 0.01, 100);
    camera.position.set(0, 0, 1.5);

    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(container.clientWidth, container.clientHeight);
    renderer.setPixelRatio(window.devicePixelRatio);
    container.appendChild(renderer.domElement);

    controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.1;
    controls.mouseButtons = {
        LEFT: null,
        MIDDLE: THREE.MOUSE.DOLLY,
        RIGHT: THREE.MOUSE.ROTATE,
    };
    controls.minDistance = 1.02;
    controls.maxDistance = 10;

    const ambient = new THREE.AmbientLight(0xffffff, 1.0);
    scene.add(ambient);

    const rcGeo = new THREE.SphereGeometry(1, 64, 32);
    const rcMat = new THREE.MeshBasicMaterial({ visible: false, side: THREE.DoubleSide });
    raycastSphere = new THREE.Mesh(rcGeo, rcMat);
    scene.add(raycastSphere);

    raycaster = new THREE.Raycaster();
    mouse = new THREE.Vector2();

    handleGroup = new THREE.Group();
    scene.add(handleGroup);

    outlineGroup = new THREE.Group();
    scene.add(outlineGroup);

    brushCursor = document.createElement('div');
    brushCursor.style.cssText = 'position:fixed;border:2px solid yellow;border-radius:50%;pointer-events:none;display:none;box-sizing:border-box;';
    document.body.appendChild(brushCursor);

    window.addEventListener('resize', onResize);
    animate();
}

function onResize() {
    const w = container.clientWidth;
    const h = container.clientHeight;
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
    renderer.setSize(w, h);
}

function animate() {
    requestAnimationFrame(animate);
    controls.update();
    renderer.render(scene, camera);
}

function createSphere() {
    const scale = eqWidth / uncropW;
    texW = eqWidth;
    texH = Math.round(uncropH * scale);
    cropPxX = Math.round(cropX0 * scale);
    cropPxY = Math.round(cropY0 * scale);

    compositeCanvas = document.createElement('canvas');
    compositeCanvas.width = texW;
    compositeCanvas.height = texH;
    compositeCtx = compositeCanvas.getContext('2d');

    overlayCanvas = document.createElement('canvas');
    overlayCanvas.width = texW;
    overlayCanvas.height = texH;
    overlayCtx = overlayCanvas.getContext('2d');

    compositeFromLabels();
    renderOverlay();

    compositeTexture = new THREE.CanvasTexture(compositeCanvas);
    compositeTexture.colorSpace = THREE.SRGBColorSpace;
    const sphereGeo = new THREE.SphereGeometry(1, 128, 64, -Math.PI / 2);
    const sphereMat = new THREE.MeshBasicMaterial({ map: compositeTexture });
    sphereMesh = new THREE.Mesh(sphereGeo, sphereMat);
    scene.add(sphereMesh);

    overlayTexture = new THREE.CanvasTexture(overlayCanvas);
    const overlayMat = new THREE.MeshBasicMaterial({
        map: overlayTexture,
        transparent: true,
        depthWrite: false,
    });
    const overlayGeo = new THREE.SphereGeometry(1.002, 128, 64, -Math.PI / 2);
    overlayMesh = new THREE.Mesh(overlayGeo, overlayMat);
    overlayMesh.visible = showOverlay;
    scene.add(overlayMesh);
}

function compositeFromLabels() {
    compositeCtx.fillStyle = '#111';
    compositeCtx.fillRect(0, 0, texW, texH);

    const out = compositeCtx.createImageData(eqWidth, eqHeight);
    const outData = out.data;

    for (let y = 0; y < eqHeight; y++) {
        for (let x = 0; x < eqWidth; x++) {
            const idx = y * eqWidth + x;
            const label = seamLabels[idx];
            const outOff = idx * 4;

            if (label < layers.length && label !== 255) {
                const layerData = layers[label].data;
                const maskVal = masks[label][idx];
                if (maskVal > 0) {
                    outData[outOff] = layerData[idx * 4];
                    outData[outOff + 1] = layerData[idx * 4 + 1];
                    outData[outOff + 2] = layerData[idx * 4 + 2];
                    outData[outOff + 3] = 255;
                    continue;
                }
            }

            for (let i = 0; i < layers.length; i++) {
                if (masks[i][idx] > 0) {
                    const ld = layers[i].data;
                    outData[outOff] = ld[idx * 4];
                    outData[outOff + 1] = ld[idx * 4 + 1];
                    outData[outOff + 2] = ld[idx * 4 + 2];
                    outData[outOff + 3] = 255;
                    break;
                }
            }
        }
    }

    compositeCtx.putImageData(out, cropPxX, cropPxY);
}

function renderOverlay() {
    overlayCtx.clearRect(0, 0, texW, texH);

    const out = overlayCtx.createImageData(eqWidth, eqHeight);
    const outData = out.data;

    for (let y = 0; y < eqHeight; y++) {
        for (let x = 0; x < eqWidth; x++) {
            const idx = y * eqWidth + x;
            const label = seamLabels[idx];
            const outOff = idx * 4;

            if (label < COLORS.length && label !== 255) {
                const c = COLORS[label % COLORS.length];
                outData[outOff] = c[0];
                outData[outOff + 1] = c[1];
                outData[outOff + 2] = c[2];
                outData[outOff + 3] = 60;
            }
        }
    }

    overlayCtx.putImageData(out, cropPxX, cropPxY);
}

function createHandles() {
    handleGroup.clear();

    for (let i = 0; i < meta.n_images; i++) {
        const img = meta.images[i];
        const pos = yawPitchToSphere(img.yaw, img.pitch);
        const c = COLORS[i % COLORS.length];

        const handleGeo = new THREE.SphereGeometry(0.025, 16, 16);
        const handleMat = new THREE.MeshBasicMaterial({
            color: new THREE.Color(c[0] / 255, c[1] / 255, c[2] / 255),
        });
        const handle = new THREE.Mesh(handleGeo, handleMat);
        handle.position.copy(pos.multiplyScalar(1.03));
        handle.userData.imageIndex = i;
        handleGroup.add(handle);

        const canvas = document.createElement('canvas');
        canvas.width = 64;
        canvas.height = 64;
        const ctx = canvas.getContext('2d');
        ctx.fillStyle = `rgb(${c[0]},${c[1]},${c[2]})`;
        ctx.beginPath();
        ctx.arc(32, 32, 28, 0, Math.PI * 2);
        ctx.fill();
        ctx.strokeStyle = 'white';
        ctx.lineWidth = 3;
        ctx.stroke();
        ctx.fillStyle = 'white';
        ctx.font = 'bold 28px sans-serif';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillText(String(i), 32, 33);

        const tex = new THREE.CanvasTexture(canvas);
        const spriteMat = new THREE.SpriteMaterial({ map: tex, depthTest: false });
        const sprite = new THREE.Sprite(spriteMat);
        sprite.scale.set(0.06, 0.06, 1);
        sprite.position.copy(handle.position);
        sprite.position.multiplyScalar(1.03);
        sprite.userData.imageIndex = i;
        handleGroup.add(sprite);
    }
}

function createOutlines() {
    outlineGroup.clear();

    const hFovDeg = meta.h_fov * 180 / Math.PI;
    const vFovDeg = meta.v_fov * 180 / Math.PI;

    for (let i = 0; i < meta.n_images; i++) {
        const img = meta.images[i];
        const c = COLORS[i % COLORS.length];

        const halfH = hFovDeg / 2;
        const halfV = vFovDeg / 2;
        const steps = 32;
        const points = [];

        for (let s = 0; s <= steps; s++) {
            const yaw = img.yaw - halfH + (hFovDeg * s / steps);
            points.push(yawPitchToSphere(yaw, img.pitch + halfV).multiplyScalar(1.005));
        }
        for (let s = 0; s <= steps; s++) {
            const pitch = img.pitch + halfV - (vFovDeg * s / steps);
            points.push(yawPitchToSphere(img.yaw + halfH, pitch).multiplyScalar(1.005));
        }
        for (let s = 0; s <= steps; s++) {
            const yaw = img.yaw + halfH - (hFovDeg * s / steps);
            points.push(yawPitchToSphere(yaw, img.pitch - halfV).multiplyScalar(1.005));
        }
        for (let s = 0; s <= steps; s++) {
            const pitch = img.pitch - halfV + (vFovDeg * s / steps);
            points.push(yawPitchToSphere(img.yaw - halfH, pitch).multiplyScalar(1.005));
        }

        const geo = new THREE.BufferGeometry().setFromPoints(points);
        const mat = new THREE.LineBasicMaterial({
            color: new THREE.Color(c[0] / 255, c[1] / 255, c[2] / 255),
            transparent: true,
            opacity: i === selectedImage ? 1.0 : 0.4,
            linewidth: 1,
        });
        const line = new THREE.LineLoop(geo, mat);
        line.userData.imageIndex = i;
        outlineGroup.add(line);
    }
}

function updateOutlineOpacity() {
    outlineGroup.children.forEach(line => {
        if (line.material) {
            line.material.opacity = line.userData.imageIndex === selectedImage ? 1.0 : 0.4;
        }
    });
}

function updateHandlePositions() {
    const children = handleGroup.children;
    for (let c = 0; c < children.length; c++) {
        const child = children[c];
        const i = child.userData.imageIndex;
        if (i === undefined) continue;
        const img = meta.images[i];
        const pos = yawPitchToSphere(img.yaw, img.pitch).multiplyScalar(1.03);
        child.position.copy(pos);
        if (child.isSprite) {
            child.position.multiplyScalar(1.03);
        }
    }
}

function hitTestImage(event) {
    const rect = renderer.domElement.getBoundingClientRect();
    mouse.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
    mouse.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;

    raycaster.setFromCamera(mouse, camera);
    const targets = overlayMesh && overlayMesh.visible
        ? [sphereMesh, overlayMesh] : [sphereMesh];
    const intersects = raycaster.intersectObjects(targets, false);
    if (intersects.length === 0) { lastHitPoint = null; return -1; }

    lastHitPoint = intersects[0].point.normalize();
    const { yaw, pitch } = sphereToYawPitch(lastHitPoint);
    const { x: rawEx, y: rawEy } = yawPitchToEquirect(yaw, pitch);
    const ex = Math.round(rawEx);
    const ey = Math.round(rawEy);

    if (selectedImage >= 0) {
        const off = imageOffsets[selectedImage];
        const mx = ex - off.dx;
        const my = ey - off.dy;
        if (mx >= 0 && mx < eqWidth && my >= 0 && my < eqHeight) {
            if (masks[selectedImage][my * eqWidth + mx] > 0) {
                return selectedImage;
            }
        }
    }

    if (!pendingChanges) {
        if (ex >= 0 && ex < eqWidth && ey >= 0 && ey < eqHeight) {
            const label = seamLabels[ey * eqWidth + ex];
            if (label < meta.n_images && label !== 255) return label;
        }
    }

    for (let i = 0; i < meta.n_images; i++) {
        const off = imageOffsets[i];
        const mx = ex - off.dx;
        const my = ey - off.dy;
        if (mx >= 0 && mx < eqWidth && my >= 0 && my < eqHeight) {
            if (masks[i][my * eqWidth + mx] > 0) return i;
        }
    }

    lastHitPoint = null;
    return -1;
}

function getLayersAtPoint(event) {
    const rect = renderer.domElement.getBoundingClientRect();
    mouse.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
    mouse.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;

    raycaster.setFromCamera(mouse, camera);
    const intersects = raycaster.intersectObjects([raycastSphere], false);
    if (intersects.length === 0) return [];

    const point = intersects[0].point.normalize();
    lastHitPoint = point;
    const { yaw, pitch } = sphereToYawPitch(point);
    const { x: rawEx, y: rawEy } = yawPitchToEquirect(yaw, pitch);
    const ex = Math.round(rawEx);
    const ey = Math.round(rawEy);

    const result = [];
    const topLabel = (ex >= 0 && ex < eqWidth && ey >= 0 && ey < eqHeight)
        ? seamLabels[ey * eqWidth + ex] : 255;
    if (topLabel < meta.n_images && topLabel !== 255) {
        result.push(topLabel);
    }

    for (let i = 0; i < meta.n_images; i++) {
        if (i === topLabel) continue;
        const off = imageOffsets[i];
        const mx = ex - off.dx;
        const my = ey - off.dy;
        if (mx >= 0 && mx < eqWidth && my >= 0 && my < eqHeight) {
            if (masks[i][my * eqWidth + mx] > 0) result.push(i);
        }
    }
    return result;
}

function hitTestSphere(event) {
    const rect = renderer.domElement.getBoundingClientRect();
    mouse.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
    mouse.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;

    raycaster.setFromCamera(mouse, camera);
    const intersects = raycaster.intersectObjects([raycastSphere], false);
    if (intersects.length > 0) {
        return intersects[0].point.normalize();
    }
    return null;
}

function bringLayerToTop(layerIdx, hitPoint) {
    if (!hitPoint) return;
    const { yaw, pitch } = sphereToYawPitch(hitPoint);
    const { x: ex, y: ey } = yawPitchToEquirect(yaw, pitch);
    const cx = Math.round(ex);
    const cy = Math.round(ey);

    const r = Math.max(brushSize, 30);
    let changed = false;
    for (let dy = -r; dy <= r; dy++) {
        for (let dx = -r; dx <= r; dx++) {
            if (dx * dx + dy * dy > r * r) continue;
            const px = cx + dx;
            const py = cy + dy;
            if (px < 0 || px >= eqWidth || py < 0 || py >= eqHeight) continue;
            const idx = py * eqWidth + px;
            if (masks[layerIdx][idx] === 0) continue;
            if (seamLabels[idx] !== layerIdx) {
                seamLabels[idx] = layerIdx;
                changed = true;
            }
        }
    }
    if (changed) {
        updateCompositeRegion(cx, cy, r);
        compositeTexture.needsUpdate = true;
        if (showOverlay) {
            updateOverlayRegion(cx, cy, r);
            overlayTexture.needsUpdate = true;
        }
    }
}

function bringImageToForeground(layerIdx) {
    let changed = false;
    for (let idx = 0; idx < eqWidth * eqHeight; idx++) {
        if (masks[layerIdx][idx] > 0 && seamLabels[idx] !== layerIdx) {
            seamLabels[idx] = layerIdx;
            changed = true;
        }
    }
    if (changed) {
        compositeFromLabels();
        compositeTexture.needsUpdate = true;
        if (showOverlay) {
            renderOverlay();
            overlayTexture.needsUpdate = true;
        }
    }
}

function updateBrushCursor(event, point) {
    if (!point || mode !== 'paint') {
        brushCursor.style.display = 'none';
        return;
    }
    const angularRadius = (brushSize / eqWidth) * 2 * Math.PI;
    const edge = point.clone();
    const up = new THREE.Vector3(0, 1, 0);
    if (Math.abs(edge.dot(up)) > 0.99) up.set(1, 0, 0);
    const perp = new THREE.Vector3().crossVectors(edge, up).normalize();
    edge.applyAxisAngle(perp, angularRadius);
    const center = point.clone().project(camera);
    const edgeProj = edge.project(camera);
    const rect = renderer.domElement.getBoundingClientRect();
    const cx = (center.x * 0.5 + 0.5) * rect.width + rect.left;
    const cy = (-center.y * 0.5 + 0.5) * rect.height + rect.top;
    const ex = (edgeProj.x * 0.5 + 0.5) * rect.width + rect.left;
    const ey = (-edgeProj.y * 0.5 + 0.5) * rect.height + rect.top;
    const screenR = Math.hypot(ex - cx, ey - cy);
    const size = Math.max(screenR * 2, 6);
    brushCursor.style.display = 'block';
    brushCursor.style.width = size + 'px';
    brushCursor.style.height = size + 'px';
    brushCursor.style.left = (event.clientX - size / 2) + 'px';
    brushCursor.style.top = (event.clientY - size / 2) + 'px';
}

function paintAtSpherePoint(point, erase) {
    if (selectedImage < 0 && checkedImages.size === 0) return;
    const { yaw, pitch } = sphereToYawPitch(point);
    const { x: ex, y: ey } = yawPitchToEquirect(yaw, pitch);

    const r = brushSize;
    let changed = false;

    for (let dy = -r; dy <= r; dy++) {
        for (let dx = -r; dx <= r; dx++) {
            if (dx * dx + dy * dy > r * r) continue;
            let px = Math.round(ex + dx);
            const py = Math.round(ey + dy);
            if (py < 0 || py >= eqHeight) continue;
            if (px < 0) px += eqWidth;
            else if (px >= eqWidth) px -= eqWidth;

            const idx = py * eqWidth + px;
            if (erase) {
                const orig = originalSeamLabels[idx];
                if (seamLabels[idx] !== orig) {
                    seamLabels[idx] = orig;
                    changed = true;
                }
            } else if (checkedImages.size > 0) {
                let assigned = -1;
                for (const ci of checkedImages) {
                    if (masks[ci][idx] > 0) { assigned = ci; break; }
                }
                if (assigned >= 0 && seamLabels[idx] !== assigned) {
                    seamLabels[idx] = assigned;
                    changed = true;
                }
            } else {
                if (masks[selectedImage][idx] === 0) continue;
                if (seamLabels[idx] !== selectedImage) {
                    seamLabels[idx] = selectedImage;
                    changed = true;
                }
            }
        }
    }

    if (changed) {
        updateCompositeRegion(Math.round(ex), Math.round(ey), r);
        compositeTexture.needsUpdate = true;
        if (showOverlay) {
            updateOverlayRegion(Math.round(ex), Math.round(ey), r);
            overlayTexture.needsUpdate = true;
        }
    }
}

function updateCompositeRegion(cx, cy, r) {
    const x0 = Math.max(0, cx - r - 1);
    const y0 = Math.max(0, cy - r - 1);
    const x1 = Math.min(eqWidth - 1, cx + r + 1);
    const y1 = Math.min(eqHeight - 1, cy + r + 1);
    const w = x1 - x0 + 1;
    const h = y1 - y0 + 1;

    const imgData = compositeCtx.createImageData(w, h);
    const d = imgData.data;

    for (let y = y0; y <= y1; y++) {
        for (let x = x0; x <= x1; x++) {
            const idx = y * eqWidth + x;
            const label = seamLabels[idx];
            const outOff = ((y - y0) * w + (x - x0)) * 4;

            if (label < layers.length && label !== 255) {
                if (masks[label][idx] > 0) {
                    d[outOff] = layers[label].data[idx * 4];
                    d[outOff + 1] = layers[label].data[idx * 4 + 1];
                    d[outOff + 2] = layers[label].data[idx * 4 + 2];
                    d[outOff + 3] = 255;
                    continue;
                }
            }

            for (let i = 0; i < layers.length; i++) {
                if (masks[i][idx] > 0) {
                    d[outOff] = layers[i].data[idx * 4];
                    d[outOff + 1] = layers[i].data[idx * 4 + 1];
                    d[outOff + 2] = layers[i].data[idx * 4 + 2];
                    d[outOff + 3] = 255;
                    break;
                }
            }
        }
    }

    compositeCtx.putImageData(imgData, cropPxX + x0, cropPxY + y0);
}

function updateOverlayRegion(cx, cy, r) {
    const x0 = Math.max(0, cx - r - 1);
    const y0 = Math.max(0, cy - r - 1);
    const x1 = Math.min(eqWidth - 1, cx + r + 1);
    const y1 = Math.min(eqHeight - 1, cy + r + 1);
    const w = x1 - x0 + 1;
    const h = y1 - y0 + 1;

    const imgData = overlayCtx.createImageData(w, h);
    const d = imgData.data;

    for (let y = y0; y <= y1; y++) {
        for (let x = x0; x <= x1; x++) {
            const idx = y * eqWidth + x;
            const label = seamLabels[idx];
            const outOff = ((y - y0) * w + (x - x0)) * 4;

            if (label < COLORS.length && label !== 255) {
                const c = COLORS[label % COLORS.length];
                d[outOff] = c[0];
                d[outOff + 1] = c[1];
                d[outOff + 2] = c[2];
                d[outOff + 3] = 60;
            } else {
                d[outOff + 3] = 0;
            }
        }
    }

    overlayCtx.putImageData(imgData, cropPxX + x0, cropPxY + y0);
}

function selectImage(idx) {
    selectedImage = idx;
    document.querySelectorAll('.thumb-wrapper').forEach((el, i) => {
        el.classList.toggle('selected', i === idx);
    });
    updateOutlineOpacity();
}

function buildImageStrip() {
    const strip = document.getElementById('image-strip');
    strip.innerHTML = '';
    for (let i = 0; i < meta.n_images; i++) {
        const wrapper = document.createElement('div');
        wrapper.className = 'thumb-wrapper' + (i === selectedImage ? ' selected' : '');

        const cb = document.createElement('input');
        cb.type = 'checkbox';
        cb.className = 'thumb-check';
        cb.checked = checkedImages.has(i);
        cb.addEventListener('click', (e) => e.stopPropagation());
        cb.addEventListener('change', () => {
            if (cb.checked) {
                checkedImages.add(i);
                if (mode === 'move') bringImageToForeground(i);
            } else {
                checkedImages.delete(i);
            }
        });
        wrapper.appendChild(cb);

        const img = document.createElement('img');
        img.src = `/editor/source/${i}`;
        wrapper.appendChild(img);

        const colorDot = document.createElement('div');
        colorDot.className = 'thumb-color';
        const c = COLORS[i % COLORS.length];
        colorDot.style.background = `rgb(${c[0]},${c[1]},${c[2]})`;
        wrapper.appendChild(colorDot);

        const label = document.createElement('div');
        label.className = 'thumb-label';
        label.textContent = i;
        wrapper.appendChild(label);

        wrapper.addEventListener('click', () => selectImage(i));
        wrapper.addEventListener('dblclick', () => showLargePreview(i));
        strip.appendChild(wrapper);
    }
}

function showLargePreview(idx) {
    const overlay = document.createElement('div');
    overlay.style.cssText = 'position:fixed;inset:0;background:rgba(0,0,0,0.85);display:flex;align-items:center;justify-content:center;z-index:9999;cursor:pointer;';
    const img = document.createElement('img');
    img.src = `/editor/source/${idx}`;
    img.style.cssText = 'max-width:90vw;max-height:90vh;object-fit:contain;border-radius:4px;';
    const label = document.createElement('div');
    label.style.cssText = 'position:absolute;top:20px;left:50%;transform:translateX(-50%);color:white;font-size:16px;';
    label.textContent = `${meta.images[idx].name} (${idx})`;
    overlay.appendChild(img);
    overlay.appendChild(label);
    overlay.addEventListener('click', () => overlay.remove());
    window.addEventListener('keydown', function handler(e) {
        if (e.key === 'Escape') { overlay.remove(); window.removeEventListener('keydown', handler); }
    });
    document.body.appendChild(overlay);
}

function ensureLayerCanvases() {
    if (layerCanvases.length > 0) return;
    for (let i = 0; i < layers.length; i++) {
        const c = document.createElement('canvas');
        c.width = eqWidth;
        c.height = eqHeight;
        const ctx = c.getContext('2d');
        const imgData = ctx.createImageData(eqWidth, eqHeight);
        const d = imgData.data;
        const src = layers[i].data;
        const m = masks[i];
        for (let p = 0; p < eqWidth * eqHeight; p++) {
            if (m[p] > 0) {
                const off = p * 4;
                d[off] = src[off];
                d[off + 1] = src[off + 1];
                d[off + 2] = src[off + 2];
                d[off + 3] = 255;
            }
        }
        ctx.putImageData(imgData, 0, 0);
        layerCanvases.push(c);
    }
}

function freeLayerCanvases() {
    for (const c of layerCanvases) { c.width = 0; c.height = 0; }
    layerCanvases = [];
}

function buildDragCanvases(imageIndex) {
    dragBaseCanvas = document.createElement('canvas');
    dragBaseCanvas.width = texW;
    dragBaseCanvas.height = texH;
    const wrapCtx = dragBaseCanvas.getContext('2d');
    wrapCtx.fillStyle = '#111';
    wrapCtx.fillRect(0, 0, texW, texH);

    if (pendingChanges) {
        ensureLayerCanvases();
        for (let i = 0; i < layerCanvases.length; i++) {
            if (i === imageIndex) continue;
            const off = imageOffsets[i];
            wrapCtx.drawImage(layerCanvases[i], cropPxX + off.dx, cropPxY + off.dy);
        }
        dragLayerCanvas = layerCanvases[imageIndex];
    } else {
        const baseData = new ImageData(eqWidth, eqHeight);
        const bd = baseData.data;
        for (let y = 0; y < eqHeight; y++) {
            for (let x = 0; x < eqWidth; x++) {
                const idx = y * eqWidth + x;
                const off = idx * 4;
                let label = seamLabels[idx];
                if (label === imageIndex) label = 255;
                if (label < layers.length && label !== 255 && masks[label][idx] > 0) {
                    bd[off] = layers[label].data[off];
                    bd[off + 1] = layers[label].data[off + 1];
                    bd[off + 2] = layers[label].data[off + 2];
                    bd[off + 3] = 255;
                    continue;
                }
                for (let i = 0; i < layers.length; i++) {
                    if (i === imageIndex) continue;
                    if (masks[i][idx] > 0) {
                        bd[off] = layers[i].data[off];
                        bd[off + 1] = layers[i].data[off + 1];
                        bd[off + 2] = layers[i].data[off + 2];
                        bd[off + 3] = 255;
                        break;
                    }
                }
            }
        }
        wrapCtx.putImageData(baseData, cropPxX, cropPxY);

        dragLayerCanvas = document.createElement('canvas');
        dragLayerCanvas.width = eqWidth;
        dragLayerCanvas.height = eqHeight;
        const layerCtx = dragLayerCanvas.getContext('2d');
        const layerImgData = layerCtx.createImageData(eqWidth, eqHeight);
        const ld = layerImgData.data;
        const srcLayer = layers[imageIndex].data;
        const srcMask = masks[imageIndex];
        for (let i = 0; i < eqWidth * eqHeight; i++) {
            if (srcMask[i] > 0) {
                const off = i * 4;
                ld[off] = srcLayer[off];
                ld[off + 1] = srcLayer[off + 1];
                ld[off + 2] = srcLayer[off + 2];
                ld[off + 3] = 255;
            }
        }
        layerCtx.putImageData(layerImgData, 0, 0);
    }

    dragStartOffset = { dx: imageOffsets[imageIndex].dx, dy: imageOffsets[imageIndex].dy };
    dragStartEq = yawPitchToEquirect(
        meta.images[imageIndex].yaw, meta.images[imageIndex].pitch);
}

function updateDragTexture(imageIndex) {
    const currentEq = yawPitchToEquirect(
        meta.images[imageIndex].yaw, meta.images[imageIndex].pitch);
    const dx = Math.round(currentEq.x - dragStartEq.x) + dragStartOffset.dx;
    const dy = Math.round(currentEq.y - dragStartEq.y) + dragStartOffset.dy;

    compositeCtx.fillStyle = '#111';
    compositeCtx.fillRect(0, 0, texW, texH);
    compositeCtx.drawImage(dragBaseCanvas, 0, 0);
    compositeCtx.globalAlpha = dragOpacity;
    compositeCtx.drawImage(dragLayerCanvas, cropPxX + dx, cropPxY + dy);
    compositeCtx.globalAlpha = 1.0;
    compositeTexture.needsUpdate = true;
}

function getRotationUpdates() {
    return meta.images.map((img, i) => ({
        index: i,
        yaw: img.yaw,
        pitch: img.pitch,
        roll: img.roll,
    }));
}

async function applyRotationsAndReload() {
    try {
        showStatus('Re-projecting images...');
        const updates = getRotationUpdates();
        const resp = await fetch('/editor/apply-rotations', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ rotations: updates }),
        });
        if (!resp.ok) {
            const body = await resp.text();
            throw new Error(`Server error ${resp.status}: ${body}`);
        }

        showStatus('Reloading layers...');
        const metaResp = await fetch('/editor/data?t=' + Date.now());
        meta = await metaResp.json();
        eqWidth = meta.width;
        eqHeight = meta.height;
        const crop = meta.crop;
        if (crop) {
            uncropW = crop.uncropped_w;
            uncropH = crop.uncropped_h;
            cropX0 = crop.x0;
            cropY0 = crop.y0;
        } else {
            uncropW = meta.full_width;
            uncropH = meta.full_height;
            cropX0 = 0;
            cropY0 = 0;
        }

        layers = [];
        masks = [];
        for (let i = 0; i < meta.n_images; i++) {
            showStatus(`Reloading layer ${i + 1}/${meta.n_images}...`);
            const layerImg = await loadImage(`/editor/layer/${i}?t=${Date.now()}`);
            layers.push(getImageData(layerImg));
            const maskImg = await loadImage(`/editor/mask/${i}?t=${Date.now()}`);
            masks.push(getMaskData(maskImg));
        }

        showStatus('Reloading seams...');
        const seamsImg = await loadImage(`/editor/seams?t=${Date.now()}`);
        const seamsData = getImageData(seamsImg);
        seamLabels = new Uint8Array(eqWidth * eqHeight);
        for (let i = 0; i < seamLabels.length; i++) {
            seamLabels[i] = seamsData.data[i * 4];
        }
        originalSeamLabels = new Uint8Array(seamLabels);

        if (sphereMesh) scene.remove(sphereMesh);
        if (overlayMesh) scene.remove(overlayMesh);
        createSphere();
        createHandles();
        createOutlines();
        hideStatus();
    } catch (err) {
        console.error('applyRotationsAndReload failed:', err);
        showStatus('Failed: ' + (err.message || String(err)));
        setTimeout(hideStatus, 5000);
    }
}

function seamLabelsToCanvas() {
    const c = document.createElement('canvas');
    c.width = eqWidth;
    c.height = eqHeight;
    const ctx = c.getContext('2d');
    const imgData = ctx.createImageData(eqWidth, eqHeight);
    for (let i = 0; i < seamLabels.length; i++) {
        const v = seamLabels[i];
        imgData.data[i * 4] = v;
        imgData.data[i * 4 + 1] = v;
        imgData.data[i * 4 + 2] = v;
        imgData.data[i * 4 + 3] = 255;
    }
    ctx.putImageData(imgData, 0, 0);
    return c;
}

async function doPreview() {
    showStatus('Generating preview...');
    try {
        const seamsCanvas = seamLabelsToCanvas();
        const blob = await new Promise(r => seamsCanvas.toBlob(r, 'image/png'));
        const resp = await fetch('/editor/preview', {
            method: 'POST',
            headers: { 'Content-Type': 'image/png' },
            body: blob,
        });
        if (!resp.ok) throw new Error('Preview failed');
        const previewBlob = await resp.blob();
        const previewImg = await loadImage(URL.createObjectURL(previewBlob));
        compositeCtx.fillStyle = '#111';
        compositeCtx.fillRect(0, 0, texW, texH);
        compositeCtx.drawImage(previewImg, cropPxX, cropPxY, eqWidth, eqHeight);
        compositeTexture.needsUpdate = true;
        showStatus('Preview applied (Gaussian-blended)');
        setTimeout(hideStatus, 3000);
    } catch (err) {
        showStatus('Preview failed: ' + err.message);
        setTimeout(hideStatus, 3000);
    }
}

let renderController = null;

async function doRender() {
    const btn = document.getElementById('btn-render');
    if (renderController) {
        renderController.abort();
        renderController = null;
        fetch('/editor/cancel-render', { method: 'POST' });
        btn.textContent = 'Render Full Res';
        btn.classList.remove('apply');
        btn.classList.add('primary');
        showStatus('Render cancelled');
        setTimeout(hideStatus, 2000);
        return;
    }
    renderController = new AbortController();
    btn.textContent = 'Cancel Render';
    btn.classList.remove('primary');
    btn.classList.add('apply');
    showStatus('Rendering at full resolution... this may take a minute.');
    try {
        const seamsCanvas = seamLabelsToCanvas();
        const blob = await new Promise(r => seamsCanvas.toBlob(r, 'image/png'));
        const resp = await fetch('/editor/render', {
            method: 'POST',
            headers: { 'Content-Type': 'image/png' },
            body: blob,
            signal: renderController.signal,
        });
        if (!resp.ok) throw new Error('Render failed');
        const result = await resp.json();
        showStatus('Render complete: ' + result.path);
        setTimeout(hideStatus, 5000);
    } catch (err) {
        if (err.name !== 'AbortError') {
            showStatus('Render failed: ' + err.message);
            setTimeout(hideStatus, 3000);
        }
    } finally {
        renderController = null;
        btn.textContent = 'Render Full Res';
        btn.classList.remove('apply');
        btn.classList.add('primary');
    }
}

async function doSave() {
    showStatus('Saving...');
    try {
        const seamsCanvas = seamLabelsToCanvas();
        const blob = await new Promise(r => seamsCanvas.toBlob(r, 'image/png'));
        const resp = await fetch('/editor/save-seams', {
            method: 'POST',
            headers: { 'Content-Type': 'image/png' },
            body: blob,
        });
        if (!resp.ok) throw new Error('Save failed');

        localStorage.setItem('pano-editor', JSON.stringify({
            selectedImage,
            mode,
            brushSize,
            dragOpacity,
            showOverlay,
        }));

        showStatus('Saved');
        setTimeout(hideStatus, 1500);
    } catch (err) {
        showStatus('Save failed: ' + err.message);
        setTimeout(hideStatus, 3000);
    }
}

function restoreUIState() {
    try {
        const raw = localStorage.getItem('pano-editor');
        if (!raw) return;
        const s = JSON.parse(raw);
        if (typeof s.selectedImage === 'number' && s.selectedImage >= 0 && s.selectedImage < meta.n_images) {
            selectImage(s.selectedImage);
        }
        if (typeof s.brushSize === 'number') {
            brushSize = s.brushSize;
            document.getElementById('brush-size').value = brushSize;
            document.getElementById('brush-size-val').textContent = brushSize;
        }
        if (typeof s.dragOpacity === 'number') {
            dragOpacity = s.dragOpacity;
            document.getElementById('drag-opacity').value = Math.round(dragOpacity * 100);
            document.getElementById('drag-opacity-val').textContent = Math.round(dragOpacity * 100) + '%';
        }
        if (s.showOverlay) {
            showOverlay = true;
            overlayMesh.visible = true;
            document.getElementById('btn-overlay').classList.add('active');
            renderOverlay();
            overlayTexture.needsUpdate = true;
        }
        if (s.mode === 'paint') setMode('paint');
    } catch {}
}

async function saveProject() {
    const name = prompt('Project name:');
    if (!name) return;
    showStatus('Saving project...');
    try {
        const seamsCanvas = seamLabelsToCanvas();
        const blob = await new Promise(r => seamsCanvas.toBlob(r, 'image/png'));
        await fetch('/editor/save-seams', {
            method: 'POST',
            headers: { 'Content-Type': 'image/png' },
            body: blob,
        });
        const resp = await fetch('/projects/save', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ name }),
        });
        if (!resp.ok) {
            const text = await resp.text();
            let msg = 'Save failed';
            try { msg = JSON.parse(text).error; } catch {}
            throw new Error(msg);
        }
        showStatus(`Project saved: ${name}`);
        setTimeout(hideStatus, 2000);
    } catch (err) {
        showStatus('Save failed: ' + err.message);
        setTimeout(hideStatus, 3000);
    }
}

async function loadProject() {
    showStatus('Loading project list...');
    try {
        const resp = await fetch('/projects/list');
        const { projects } = await resp.json();
        if (projects.length === 0) {
            showStatus('No saved projects');
            setTimeout(hideStatus, 2000);
            return;
        }
        const overlay = document.createElement('div');
        overlay.style.cssText = 'position:fixed;inset:0;background:rgba(0,0,0,0.85);display:flex;align-items:center;justify-content:center;z-index:9999;';
        const panel = document.createElement('div');
        panel.style.cssText = 'background:#222;border-radius:8px;padding:20px;min-width:300px;max-height:80vh;overflow-y:auto;';
        panel.innerHTML = '<h3 style="margin:0 0 12px;color:#fff;">Load Project</h3>';
        projects.forEach(name => {
            const btn = document.createElement('button');
            btn.textContent = name;
            btn.style.cssText = 'display:block;width:100%;padding:8px 12px;margin:4px 0;background:#444;border:1px solid #666;border-radius:4px;color:#fff;cursor:pointer;text-align:left;font-size:14px;';
            btn.addEventListener('mouseenter', () => btn.style.background = '#555');
            btn.addEventListener('mouseleave', () => btn.style.background = '#444');
            btn.addEventListener('click', async () => {
                overlay.remove();
                showStatus(`Loading project: ${name}...`);
                const r = await fetch('/projects/load', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ name }),
                });
                if (!r.ok) throw new Error((await r.json()).error);
                window.location.reload();
            });
            panel.appendChild(btn);
        });
        const cancelBtn = document.createElement('button');
        cancelBtn.textContent = 'Cancel';
        cancelBtn.style.cssText = 'display:block;width:100%;padding:8px 12px;margin:12px 0 0;background:#633;border:1px solid #855;border-radius:4px;color:#fff;cursor:pointer;font-size:14px;';
        cancelBtn.addEventListener('click', () => overlay.remove());
        panel.appendChild(cancelBtn);
        overlay.appendChild(panel);
        overlay.addEventListener('click', (e) => { if (e.target === overlay) overlay.remove(); });
        document.body.appendChild(overlay);
        hideStatus();
    } catch (err) {
        showStatus('Load failed: ' + err.message);
        setTimeout(hideStatus, 3000);
    }
}

async function applyPositions() {
    freeLayerCanvases();
    await applyRotationsAndReload();
    imageOffsets = layers.map(() => ({ dx: 0, dy: 0 }));
    pendingChanges = false;
    document.getElementById('btn-apply').style.display = 'none';
}

async function resetPositions() {
    if (!pendingChanges) {
        showStatus('No pending changes');
        setTimeout(hideStatus, 1500);
        return;
    }
    showStatus('Resetting positions...');
    freeLayerCanvases();
    const metaResp = await fetch('/editor/data?t=' + Date.now());
    meta = await metaResp.json();
    imageOffsets = layers.map(() => ({ dx: 0, dy: 0 }));
    pendingChanges = false;
    document.getElementById('btn-apply').style.display = 'none';
    compositeFromLabels();
    compositeTexture.needsUpdate = true;
    createHandles();
    createOutlines();
    showStatus('Positions reset');
    setTimeout(hideStatus, 1500);
}

function setMode(newMode) {
    if (newMode === 'paint' && pendingChanges) {
        showStatus('Apply position changes before painting');
        setTimeout(hideStatus, 2000);
        return;
    }
    mode = newMode;
    document.getElementById('btn-mode-move').classList.toggle('active', mode === 'move');
    document.getElementById('btn-mode-paint').classList.toggle('active', mode === 'paint');
    document.getElementById('brush-group').style.display = mode === 'paint' ? 'flex' : 'none';
    document.getElementById('mode-indicator').textContent =
        mode === 'move' ? 'Click: cycle layers | Drag: move selected | Right: orbit' : 'Left: paint | Shift: unpaint | Alt+click: pick layer | Right: orbit';

    handleGroup.visible = mode === 'move';
    brushCursor.style.display = 'none';
    renderer.domElement.style.cursor = mode === 'paint' ? 'none' : 'default';
}

function setupEvents() {
    document.getElementById('btn-mode-move').addEventListener('click', () => setMode('move'));
    document.getElementById('btn-mode-paint').addEventListener('click', () => setMode('paint'));

    const brushSlider = document.getElementById('brush-size');
    const brushVal = document.getElementById('brush-size-val');
    brushSlider.addEventListener('input', () => {
        brushSize = parseInt(brushSlider.value);
        brushVal.textContent = brushSize;
    });

    const opacitySlider = document.getElementById('drag-opacity');
    const opacityVal = document.getElementById('drag-opacity-val');
    opacitySlider.addEventListener('input', () => {
        dragOpacity = parseInt(opacitySlider.value) / 100;
        opacityVal.textContent = opacitySlider.value + '%';
        if (draggingHandle >= 0) {
            updateDragTexture(draggingHandle);
        }
    });

    document.getElementById('btn-overlay').addEventListener('click', () => {
        showOverlay = !showOverlay;
        overlayMesh.visible = showOverlay;
        document.getElementById('btn-overlay').classList.toggle('active', showOverlay);
        if (showOverlay) {
            renderOverlay();
            overlayTexture.needsUpdate = true;
        }
    });

    document.getElementById('btn-reset').addEventListener('click', resetPositions);
    document.getElementById('btn-apply').addEventListener('click', applyPositions);
    document.getElementById('btn-save').addEventListener('click', doSave);
    document.getElementById('btn-project-save').addEventListener('click', saveProject);
    document.getElementById('btn-project-load').addEventListener('click', loadProject);
    document.getElementById('btn-preview').addEventListener('click', doPreview);
    document.getElementById('btn-render').addEventListener('click', doRender);
    document.getElementById('btn-view').addEventListener('click', () => {
        window.location.href = '/';
    });

    const el = renderer.domElement;

    el.addEventListener('mousedown', (e) => {
        if (e.button !== 0) return;

        if (mode === 'move') {
            const layersHere = getLayersAtPoint(e);
            if (layersHere.length === 0) return;

            moveClickStart = {
                x: e.clientX, y: e.clientY,
                layers: layersHere,
                hitPoint: lastHitPoint ? lastHitPoint.clone() : null,
            };
            moveDragging = false;
            controls.enabled = false;
            e.preventDefault();
            return;
        }

        if (mode === 'paint') {
            if (e.altKey) {
                const layers = getLayersAtPoint(e);
                if (layers.length > 0) {
                    const currentIdx = layers.indexOf(selectedImage);
                    const nextIdx = (currentIdx + 1) % layers.length;
                    selectImage(layers[nextIdx]);
                    showStatus(`Painting with: ${meta.images[layers[nextIdx]].name}`);
                    setTimeout(hideStatus, 1500);
                }
                e.preventDefault();
                return;
            }
            if (selectedImage < 0 && checkedImages.size === 0) {
                showStatus('Check images or Alt+click to pick a layer');
                setTimeout(hideStatus, 2000);
                return;
            }
            painting = true;
            controls.enabled = false;
            const point = hitTestSphere(e);
            if (point) paintAtSpherePoint(point, e.shiftKey);
            e.preventDefault();
        }
    });

    el.addEventListener('mousemove', (e) => {
        if (moveClickStart && !moveDragging) {
            const dx = e.clientX - moveClickStart.x;
            const dy = e.clientY - moveClickStart.y;
            if (dx * dx + dy * dy > 16) {
                moveDragging = true;
                const target = selectedImage >= 0 && moveClickStart.layers.includes(selectedImage)
                    ? selectedImage : moveClickStart.layers[0];
                draggingHandle = target;
                selectImage(target);
                dragStartPoint = moveClickStart.hitPoint.clone();
                dragStartYawPitch = {
                    yaw: meta.images[target].yaw,
                    pitch: meta.images[target].pitch,
                };
                buildDragCanvases(target);
            }
            return;
        }

        if (draggingHandle >= 0 && dragStartPoint) {
            const spherePoint = hitTestSphere(e);
            if (spherePoint) {
                const currentYP = sphereToYawPitch(spherePoint);
                const startYP = sphereToYawPitch(dragStartPoint);
                const deltaYaw = currentYP.yaw - startYP.yaw;
                const deltaPitch = currentYP.pitch - startYP.pitch;

                meta.images[draggingHandle].yaw = dragStartYawPitch.yaw + deltaYaw;
                meta.images[draggingHandle].pitch = dragStartYawPitch.pitch + deltaPitch;

                updateHandlePositions();
                createOutlines();
                updateDragTexture(draggingHandle);
            }
            return;
        }

        if (painting) {
            const point = hitTestSphere(e);
            if (point) {
                paintAtSpherePoint(point, e.shiftKey);
                updateBrushCursor(e, point);
            }
            return;
        }

        if (mode === 'paint') {
            const point = hitTestSphere(e);
            updateBrushCursor(e, point);
            return;
        }

        if (mode === 'move') {
            const hit = hitTestImage(e);
            el.style.cursor = hit >= 0 ? 'grab' : 'default';
        }
    });

    el.addEventListener('mouseleave', () => {
        brushCursor.style.display = 'none';
    });

    window.addEventListener('mouseup', () => {
        if (moveClickStart && !moveDragging) {
            const layersHere = moveClickStart.layers;
            const hitPoint = moveClickStart.hitPoint;
            if (layersHere.length > 0) {
                const currentIdx = layersHere.indexOf(selectedImage);
                if (currentIdx < 0) {
                    selectImage(layersHere[0]);
                } else {
                    const nextIdx = (currentIdx + 1) % layersHere.length;
                    const nextLayer = layersHere[nextIdx];
                    selectImage(nextLayer);
                    bringLayerToTop(nextLayer, hitPoint);
                }
            }
            moveClickStart = null;
            controls.enabled = true;
            return;
        }
        moveClickStart = null;
        moveDragging = false;

        if (draggingHandle >= 0) {
            const wasDragging = draggingHandle;
            const moved = dragStartYawPitch &&
                (Math.abs(meta.images[wasDragging].yaw - dragStartYawPitch.yaw) > 0.1 ||
                 Math.abs(meta.images[wasDragging].pitch - dragStartYawPitch.pitch) > 0.1);
            if (moved && dragStartEq) {
                const currentEq = yawPitchToEquirect(
                    meta.images[wasDragging].yaw, meta.images[wasDragging].pitch);
                imageOffsets[wasDragging] = {
                    dx: Math.round(currentEq.x - dragStartEq.x) + dragStartOffset.dx,
                    dy: Math.round(currentEq.y - dragStartEq.y) + dragStartOffset.dy,
                };
                pendingChanges = true;
                document.getElementById('btn-apply').style.display = '';
            }
            draggingHandle = -1;
            dragStartPoint = null;
            dragStartYawPitch = null;
            if (dragBaseCanvas) { dragBaseCanvas.width = 0; dragBaseCanvas.height = 0; }
            dragBaseCanvas = null;
            if (dragLayerCanvas && !pendingChanges) { dragLayerCanvas.width = 0; dragLayerCanvas.height = 0; }
            dragLayerCanvas = null;
            dragStartEq = null;
            dragStartOffset = null;
            controls.enabled = true;
        }
        if (painting) {
            painting = false;
            controls.enabled = true;
        }
    });

    window.addEventListener('keydown', (e) => {
        if (e.key === 's' && (e.ctrlKey || e.metaKey)) {
            e.preventDefault();
            doSave();
            return;
        }
        if (e.key === 'Escape') {
            selectImage(-1);
            return;
        }
        if (e.key === 'm' || e.key === 'M') setMode('move');
        if (e.key === 'p' || e.key === 'P') setMode('paint');
        if (e.key === 'o' || e.key === 'O') {
            showOverlay = !showOverlay;
            overlayMesh.visible = showOverlay;
            document.getElementById('btn-overlay').classList.toggle('active', showOverlay);
            if (showOverlay) {
                renderOverlay();
                overlayTexture.needsUpdate = true;
            }
        }
        if (e.key >= '0' && e.key <= '9') {
            const idx = parseInt(e.key);
            if (idx < meta.n_images) selectImage(idx);
        }
    });
}

// Shown in place of the spinner when the editor cannot be opened at all, so the
// page explains itself instead of hanging on "Loading metadata...".
function showLoadFailure(message, detail) {
    document.querySelector('#loading-overlay .spinner').style.display = 'none';
    loadingText.innerHTML =
        `<div style="max-width:640px;text-align:center;line-height:1.5">` +
        `<div style="font-size:16px;margin-bottom:12px">${message}</div>` +
        (detail ? `<div style="opacity:0.7;font-size:13px">${detail}</div>` : '') +
        `<div style="margin-top:20px"><a href="/" style="color:#6af">` +
        `Back to viewer</a></div></div>`;
    loadingOverlay.classList.remove('hidden');
}

// Fetches the editor metadata, building the editor data first if the server has
// the source images but has not processed them yet (a plain `main.py <folder>`
// session). Returns null if the editor cannot be opened.
async function fetchEditorData() {
    let resp = await fetch('/editor/data');
    if (resp.ok) return await resp.json();
    if (resp.status !== 404) {
        showLoadFailure('Could not load editor data.',
            `The server returned ${resp.status} ${resp.statusText}.`);
        return null;
    }

    const status = await resp.json().catch(() => ({}));
    if (!status.preparable) {
        showLoadFailure('The editor is not available for this session.',
            status.reason || 'Relaunch with a folder of photos and -e.');
        return null;
    }

    loadingText.textContent =
        `Preparing editor data for ${status.n_images} images — this takes a ` +
        `few minutes. Progress is printed in the terminal.`;
    const prep = await fetch('/editor/prepare', { method: 'POST' });
    if (!prep.ok) {
        const err = await prep.json().catch(() => ({}));
        showLoadFailure('Could not prepare editor data.',
            err.error || err.reason || `Server returned ${prep.status}.`);
        return null;
    }

    loadingText.textContent = 'Loading metadata...';
    resp = await fetch('/editor/data');
    if (!resp.ok) {
        showLoadFailure('Could not load editor data.',
            `The server returned ${resp.status} after preparing it.`);
        return null;
    }
    return await resp.json();
}

async function init() {
    loadingText.textContent = 'Loading metadata...';
    meta = await fetchEditorData();
    if (meta === null) return;
    eqWidth = meta.width;
    eqHeight = meta.height;

    const crop = meta.crop;
    if (crop) {
        uncropW = crop.uncropped_w;
        uncropH = crop.uncropped_h;
        cropX0 = crop.x0;
        cropY0 = crop.y0;
    } else {
        uncropW = meta.full_width;
        uncropH = meta.full_height;
    }

    initThree();

    loadingText.textContent = 'Loading layers...';
    for (let i = 0; i < meta.n_images; i++) {
        loadingText.textContent = `Loading layers (${i + 1}/${meta.n_images})...`;
        const layerImg = await loadImage(`/editor/layer/${i}`);
        layers.push(getImageData(layerImg));
        const maskImg = await loadImage(`/editor/mask/${i}`);
        masks.push(getMaskData(maskImg));
    }

    loadingText.textContent = 'Loading seam labels...';
    const seamsImg = await loadImage('/editor/seams');
    const seamsData = getImageData(seamsImg);
    seamLabels = new Uint8Array(eqWidth * eqHeight);
    for (let i = 0; i < seamLabels.length; i++) {
        seamLabels[i] = seamsData.data[i * 4];
    }
    originalSeamLabels = new Uint8Array(seamLabels);

    loadingText.textContent = 'Building sphere...';
    imageOffsets = layers.map(() => ({ dx: 0, dy: 0 }));
    createSphere();
    createHandles();
    createOutlines();
    buildImageStrip();
    setupEvents();
    restoreUIState();

    loadingOverlay.classList.add('hidden');
}

// Anything that throws past here — a layer that 404s, a decode failure — would
// otherwise leave the overlay sitting on whatever step it reached, looking like a
// hang. Surface it instead: the message names the step that actually failed.
init().catch((err) => {
    console.error('Editor failed to initialise:', err);
    showLoadFailure('The editor could not finish loading.',
        (err && err.message) || String(err));
});
