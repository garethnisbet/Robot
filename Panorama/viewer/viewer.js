import * as THREE from "three";

const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(75, innerWidth / innerHeight, 0.1, 1000);
camera.position.set(0, 0, 0.01);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(devicePixelRatio);
renderer.setSize(innerWidth, innerHeight);
document.body.appendChild(renderer.domElement);

const geo = new THREE.SphereGeometry(50, 64, 32);
geo.scale(-1, 1, 1);

const loader = new THREE.TextureLoader();
const loadingEl = document.getElementById("loading");

let sphereMesh = null;
let panoTexture = null;

// --- Colour grading ---
// Both render paths (sphere and stereographic shader) share these uniform
// objects, so an adjustment applies in every mode and to exports.
const grade = {
    brightness: { value: 1.0 },
    contrast: { value: 1.0 },
    saturation: { value: 1.0 },
    warmth: { value: 0.0 },
};

// Grading happens in an approximately perceptual space so the sliders behave
// like a photo editor rather than pushing highlights around.
const GRADE_GLSL = `
uniform float brightness;
uniform float contrast;
uniform float saturation;
uniform float warmth;

vec3 applyGrade(vec3 linearColor) {
    vec3 c = pow(max(linearColor, 0.0), vec3(1.0 / 2.2));
    c *= brightness;
    c = (c - 0.5) * contrast + 0.5;
    float luma = dot(c, vec3(0.2126, 0.7152, 0.0722));
    c = mix(vec3(luma), c, saturation);
    c += vec3(warmth, warmth * 0.15, -warmth) * 0.15;
    return pow(max(c, 0.0), vec3(2.2));
}
`;

function attachGrade(material) {
    material.onBeforeCompile = (shader) => {
        shader.uniforms.brightness = grade.brightness;
        shader.uniforms.contrast = grade.contrast;
        shader.uniforms.saturation = grade.saturation;
        shader.uniforms.warmth = grade.warmth;
        shader.fragmentShader = GRADE_GLSL + shader.fragmentShader.replace(
            "#include <colorspace_fragment>",
            "gl_FragColor.rgb = applyGrade(gl_FragColor.rgb);\n#include <colorspace_fragment>"
        );
    };
}

// Pole repair, shared by every shader path that samples the panorama directly.
// Rows within poleFill of either pole are blended towards the average of the
// last good ring of pixels, which hides the nadir hole a drone leaves under
// itself. Sampling runs in the texture's own vertical sense, where v = 0 is the
// nadir and v = 1 the zenith; both callers map latitude to v that way.
const poleFill = { value: 0.0 };

const SAMPLE_GLSL = `
uniform sampler2D panoTexture;
uniform float poleFill;

vec4 samplePanorama(vec2 texUV) {
    if (poleFill <= 0.0) return texture2D(panoTexture, texUV);

    float v = texUV.y;
    float threshold = poleFill;

    bool nearNadir = v < threshold;
    bool nearZenith = v > 1.0 - threshold;

    if (!nearNadir && !nearZenith) return texture2D(panoTexture, texUV);

    float edgeV = nearNadir ? threshold : 1.0 - threshold;
    float dist = nearNadir ? v : 1.0 - v;
    float blend = smoothstep(0.0, threshold, dist);

    vec4 ringColor = vec4(0.0);
    for (int i = 0; i < 32; i++) {
        float u = float(i) / 32.0;
        ringColor += texture2D(panoTexture, vec2(u, edgeV));
    }
    ringColor /= 32.0;

    return mix(ringColor, texture2D(panoTexture, texUV), blend);
}
`;

// --- Stereographic projection modes ---
// 0 = normal, 1 = tiny planet (ground center), 2 = inverse (sky center)
let stereoMode = 0;
const STEREO_LABELS = ["normal", "planet", "tunnel"];

const tpScene = new THREE.Scene();
const tpCamera = new THREE.OrthographicCamera(-1, 1, 1, -1, 0, 1);

const tpMaterial = new THREE.ShaderMaterial({
    uniforms: {
        panoTexture: { value: null },
        resolution: { value: new THREE.Vector2(innerWidth * devicePixelRatio, innerHeight * devicePixelRatio) },
        rotation: { value: 0.0 },
        tilt: { value: 0.0 },
        zoom: { value: 1.0 },
        invertY: { value: 1.0 },
        stretch: { value: 1.0 },
        poleFill: poleFill,
        brightness: grade.brightness,
        contrast: grade.contrast,
        saturation: grade.saturation,
        warmth: grade.warmth,
    },
    vertexShader: `
        void main() {
            gl_Position = vec4(position.xy, 0.0, 1.0);
        }
    `,
    fragmentShader: `
        precision highp float;
        uniform vec2 resolution;
        uniform float rotation;
        uniform float tilt;
        uniform float zoom;
        uniform float invertY;
        uniform float stretch;
        ${SAMPLE_GLSL}
        ${GRADE_GLSL}

        #define PI 3.14159265359
        #define TWO_PI 6.28318530718

        void main() {
            vec2 uv = (gl_FragCoord.xy / resolution - 0.5) * 2.0;
            uv.x *= resolution.x / resolution.y;
            uv *= 4.0 / zoom;

            float r = length(uv);
            float phi = atan(uv.y, uv.x);
            r = pow(r, stretch);
            float theta = 2.0 * atan(r * 0.5);

            vec3 dir = vec3(
                sin(theta) * cos(phi),
                invertY * -cos(theta),
                sin(theta) * sin(phi)
            );

            float ct = cos(tilt), st = sin(tilt);
            vec3 tilted = vec3(dir.x, ct*dir.y - st*dir.z, st*dir.y + ct*dir.z);

            float cr = cos(rotation), sr = sin(rotation);
            vec3 rot = vec3(cr*tilted.x + sr*tilted.z, tilted.y, -sr*tilted.x + cr*tilted.z);

            float eLon = atan(rot.x, rot.z);
            float eLat = asin(clamp(rot.y, -1.0, 1.0));
            // v grows towards the zenith, matching the sphere's UVs. Negating
            // eLat here instead — as this used to — flips the projection top to
            // bottom, which swapped what the tiny planet and tunnel buttons did:
            // "ground centre" put the sky in the middle and vice versa.
            vec2 texUV = vec2(eLon / TWO_PI + 0.5, 0.5 + eLat / PI);

            vec4 sampled = samplePanorama(texUV);
            gl_FragColor = vec4(applyGrade(sampled.rgb), sampled.a);
            // Sampling an sRGB texture yields linear light, so without this the
            // stereographic modes render darker than the sphere does.
            #include <colorspace_fragment>
        }
    `,
});

tpScene.add(new THREE.Mesh(new THREE.PlaneGeometry(2, 2), tpMaterial));

// --- Equirectangular re-render ---
// Only ever used by exportEquirect: it rewrites the panorama at 2:1 with the
// grade and pole fill baked into the pixels, leaving the projection itself
// untouched. The UV mapping matches the sphere's (v = 1 at the north pole), so
// with the grade at its defaults and no pole fill the output is the input.
const eqScene = new THREE.Scene();

const eqMaterial = new THREE.ShaderMaterial({
    uniforms: {
        panoTexture: { value: null },
        resolution: { value: new THREE.Vector2(1, 1) },
        poleFill: poleFill,
        brightness: grade.brightness,
        contrast: grade.contrast,
        saturation: grade.saturation,
        warmth: grade.warmth,
    },
    vertexShader: `
        void main() {
            gl_Position = vec4(position.xy, 0.0, 1.0);
        }
    `,
    fragmentShader: `
        precision highp float;
        uniform vec2 resolution;
        ${SAMPLE_GLSL}
        ${GRADE_GLSL}

        void main() {
            vec2 texUV = gl_FragCoord.xy / resolution;
            vec4 sampled = samplePanorama(texUV);
            gl_FragColor = vec4(applyGrade(sampled.rgb), sampled.a);
            // An sRGB texture is decoded to linear on sampling, so without this
            // the readback is linear light and the exported JPEG comes out dark.
            // Needs a render target whose texture is tagged SRGBColorSpace,
            // which is what generates the conversion this include expands to.
            #include <colorspace_fragment>
        }
    `,
});

eqScene.add(new THREE.Mesh(new THREE.PlaneGeometry(2, 2), eqMaterial));

let stretchValue = 1.0;

function updateStereoUI() {
    const btnPlanet = document.getElementById("btn-tinyplanet");
    const btnTunnel = document.getElementById("btn-tunnel");
    const stretchPanel = document.getElementById("stereo-panel");
    btnPlanet.classList.toggle("active", stereoMode === 1);
    btnTunnel.classList.toggle("active", stereoMode === 2);
    stretchPanel.classList.toggle("visible", stereoMode > 0);
}

function setStereoMode(mode) {
    stereoMode = mode;
    if (stereoMode > 0) {
        tpMaterial.uniforms.invertY.value = stereoMode === 1 ? 1.0 : -1.0;
        targetLat = 0;
        targetFov = 75;
    }
    momentum = { x: 0, y: 0 };
    updateStereoUI();
}

let poleFillValue = 0.0;

function setStretch(val) {
    stretchValue = THREE.MathUtils.clamp(val, 0.3, 3.0);
    tpMaterial.uniforms.stretch.value = stretchValue;
    const slider = document.getElementById("stretch-slider");
    const display = document.getElementById("stretch-value");
    slider.value = stretchValue;
    display.textContent = stretchValue.toFixed(2);
}

function setPoleFill(val) {
    poleFillValue = THREE.MathUtils.clamp(val, 0.0, 0.25);
    poleFill.value = poleFillValue;
    const slider = document.getElementById("polefill-slider");
    const display = document.getElementById("polefill-value");
    slider.value = poleFillValue;
    display.textContent = Math.round(poleFillValue * 100) + "%";
}

const GRADE_DEFAULTS = { brightness: 1.0, contrast: 1.0, saturation: 1.0, warmth: 0.0 };
const GRADE_LIMITS = {
    brightness: [0.2, 2.0],
    contrast: [0.2, 2.0],
    saturation: [0.0, 2.0],
    warmth: [-1.0, 1.0],
};

function formatGrade(key, val) {
    return key === "warmth"
        ? (val > 0 ? "+" : "") + Math.round(val * 100)
        : Math.round(val * 100) + "%";
}

function setGrade(key, val) {
    const [min, max] = GRADE_LIMITS[key];
    const v = THREE.MathUtils.clamp(val, min, max);
    grade[key].value = v;
    document.getElementById(`${key}-slider`).value = v;
    document.getElementById(`${key}-value`).textContent = formatGrade(key, v);
}

function resetGrade() {
    for (const key of Object.keys(GRADE_DEFAULTS)) setGrade(key, GRADE_DEFAULTS[key]);
}

for (const key of Object.keys(GRADE_DEFAULTS)) {
    document.getElementById(`${key}-slider`).addEventListener("input", (e) => {
        setGrade(key, parseFloat(e.target.value));
    });
}
document.getElementById("color-reset").addEventListener("click", resetGrade);

function toggleColorPanel(force) {
    const panel = document.getElementById("color-panel");
    const visible = force !== undefined ? force : !panel.classList.contains("visible");
    panel.classList.toggle("visible", visible);
    document.getElementById("btn-color").classList.toggle("active", visible);
}

document.getElementById("btn-color").addEventListener("click", () => toggleColorPanel());

function loadPanoramaTexture(url) {
    return new Promise((resolve, reject) => {
        loader.load(
            url,
            (texture) => {
                texture.colorSpace = THREE.SRGBColorSpace;
                if (panoTexture) panoTexture.dispose();
                panoTexture = texture;
                tpMaterial.uniforms.panoTexture.value = texture;
                eqMaterial.uniforms.panoTexture.value = texture;

                if (sphereMesh) {
                    sphereMesh.material.map?.dispose();
                    sphereMesh.material.map = texture;
                    sphereMesh.material.needsUpdate = true;
                } else {
                    const mat = new THREE.MeshBasicMaterial({ map: texture });
                    attachGrade(mat);
                    sphereMesh = new THREE.Mesh(geo, mat);
                    scene.add(sphereMesh);
                }
                resolve();
            },
            undefined,
            reject
        );
    });
}

const welcomeEl = document.getElementById("welcome");

fetch("/panorama-info")
    .then(r => r.json())
    .then(info => {
        if (info.path) {
            welcomeEl.classList.add("hidden");
            setTimeout(() => welcomeEl.remove(), 600);
            return loadPanoramaTexture("/panorama-image").then(() => {
                loadingEl.classList.add("hidden");
                setTimeout(() => loadingEl.remove(), 600);
            });
        }
        loadingEl.classList.add("hidden");
        setTimeout(() => loadingEl.remove(), 600);
    })
    .catch(() => {
        loadingEl.classList.add("hidden");
        setTimeout(() => loadingEl.remove(), 600);
    });

// --- Interaction state ---
let isDragging = false;
let prevMouse = { x: 0, y: 0 };
let lon = 0, lat = 0;
let targetLon = 0, targetLat = 0;
let fov = 75, targetFov = 75;
let autoRotate = false;
let autoRotateSpeed = 0.15;
let momentum = { x: 0, y: 0 };
let lastDelta = { x: 0, y: 0 };

const FOV_MIN = 20;
const FOV_MAX = 110;
const STEREO_FOV_MIN = 10;
const STEREO_FOV_MAX = 160;
const LAT_MAX = 85;

function fovMin() { return stereoMode > 0 ? STEREO_FOV_MIN : FOV_MIN; }
function fovMax() { return stereoMode > 0 ? STEREO_FOV_MAX : FOV_MAX; }
function clampFov(v) { return THREE.MathUtils.clamp(v, fovMin(), fovMax()); }

// --- Mouse controls ---
renderer.domElement.addEventListener("pointerdown", (e) => {
    isDragging = true;
    prevMouse = { x: e.clientX, y: e.clientY };
    momentum = { x: 0, y: 0 };
    autoRotate = false;
    updateAutoRotateUI();
    renderer.domElement.style.cursor = "grabbing";
});

window.addEventListener("pointermove", (e) => {
    if (!isDragging) return;
    const dx = e.clientX - prevMouse.x;
    const dy = e.clientY - prevMouse.y;
    const scale = fov / 1200;
    targetLon -= dx * scale;
    targetLat += dy * scale;
    targetLat = THREE.MathUtils.clamp(targetLat, -LAT_MAX, LAT_MAX);
    lastDelta = { x: -dx * scale, y: dy * scale };
    prevMouse = { x: e.clientX, y: e.clientY };
});

window.addEventListener("pointerup", () => {
    if (isDragging) {
        momentum = { x: lastDelta.x, y: lastDelta.y };
    }
    isDragging = false;
    renderer.domElement.style.cursor = "grab";
});

renderer.domElement.style.cursor = "grab";

// --- Scroll zoom ---
renderer.domElement.addEventListener("wheel", (e) => {
    e.preventDefault();
    targetFov += e.deltaY * 0.05;
    targetFov = clampFov(targetFov);
}, { passive: false });

// --- Touch controls ---
let touchStartDist = 0;
let touchStartFov = 75;

renderer.domElement.addEventListener("touchstart", (e) => {
    if (e.touches.length === 1) {
        isDragging = true;
        prevMouse = { x: e.touches[0].clientX, y: e.touches[0].clientY };
        momentum = { x: 0, y: 0 };
        autoRotate = false;
        updateAutoRotateUI();
    } else if (e.touches.length === 2) {
        isDragging = false;
        touchStartDist = Math.hypot(
            e.touches[1].clientX - e.touches[0].clientX,
            e.touches[1].clientY - e.touches[0].clientY
        );
        touchStartFov = targetFov;
    }
}, { passive: false });

renderer.domElement.addEventListener("touchmove", (e) => {
    e.preventDefault();
    if (e.touches.length === 1 && isDragging) {
        const dx = e.touches[0].clientX - prevMouse.x;
        const dy = e.touches[0].clientY - prevMouse.y;
        const scale = fov / 1200;
        targetLon -= dx * scale;
        targetLat += dy * scale;
        targetLat = THREE.MathUtils.clamp(targetLat, -LAT_MAX, LAT_MAX);
        lastDelta = { x: -dx * scale, y: dy * scale };
        prevMouse = { x: e.touches[0].clientX, y: e.touches[0].clientY };
    } else if (e.touches.length === 2) {
        const dist = Math.hypot(
            e.touches[1].clientX - e.touches[0].clientX,
            e.touches[1].clientY - e.touches[0].clientY
        );
        targetFov = touchStartFov * (touchStartDist / dist);
        targetFov = clampFov(targetFov);
    }
}, { passive: false });

renderer.domElement.addEventListener("touchend", () => {
    if (isDragging) {
        momentum = { x: lastDelta.x, y: lastDelta.y };
    }
    isDragging = false;
});

// --- Keyboard ---
window.addEventListener("keydown", (e) => {
    // A focused slider handles its own arrow keys; don't pan the view as well.
    if (e.target instanceof HTMLInputElement) return;

    const step = 2;
    switch (e.key) {
        case "ArrowLeft": targetLon -= step; break;
        case "ArrowRight": targetLon += step; break;
        case "ArrowUp": targetLat = Math.min(targetLat + step, LAT_MAX); break;
        case "ArrowDown": targetLat = Math.max(targetLat - step, -LAT_MAX); break;
        case "+": case "=": targetFov = clampFov(targetFov - 5); break;
        case "-": targetFov = clampFov(targetFov + 5); break;
        case "r": resetView(); break;
        case "f": toggleFullscreen(); break;
        case "t": setStereoMode(stereoMode === 1 ? 0 : 1); break;
        case "y": setStereoMode(stereoMode === 2 ? 0 : 2); break;
        case "[": if (stereoMode > 0) setStretch(stretchValue - 0.1); break;
        case "]": if (stereoMode > 0) setStretch(stretchValue + 0.1); break;
        case "c": toggleColorPanel(); break;
        case "e": exportFullRes(); break;
        case "x": exportEquirect(); break;
        case "d": downloadPanorama(); break;
        case " ": autoRotate = !autoRotate; updateAutoRotateUI(); e.preventDefault(); break;
    }
});

// --- Buttons ---
document.getElementById("btn-zoom-in").addEventListener("click", () => {
    targetFov = clampFov(targetFov - 10);
});
document.getElementById("btn-zoom-out").addEventListener("click", () => {
    targetFov = clampFov(targetFov + 10);
});
document.getElementById("btn-reset").addEventListener("click", resetView);
document.getElementById("btn-autorotate").addEventListener("click", () => {
    autoRotate = !autoRotate;
    updateAutoRotateUI();
});
document.getElementById("btn-fullscreen").addEventListener("click", toggleFullscreen);
document.getElementById("btn-tinyplanet").addEventListener("click", () => setStereoMode(stereoMode === 1 ? 0 : 1));
document.getElementById("btn-tunnel").addEventListener("click", () => setStereoMode(stereoMode === 2 ? 0 : 2));
document.getElementById("btn-export").addEventListener("click", exportFullRes);
document.getElementById("btn-export-360").addEventListener("click", exportEquirect);
document.getElementById("btn-download").addEventListener("click", downloadPanorama);
const editorBtn = document.getElementById("btn-editor");
editorBtn.addEventListener("click", () => {
    if (editorBtn.disabled) return;
    // Versioned so a browser holding a pre-no-store copy of editor.html cannot
    // serve it from cache: that copy references an old editor.js and fails in
    // confusing ways. See the note in editor.html.
    window.location.href = '/editor.html?v=2';
});

// The editor needs the original photos, which this session may not have. Reflect
// that on the button rather than navigating to a page that cannot load. Re-run
// after an import, which can supply the sources a session started without.
function refreshEditorStatus() {
    return fetch("/editor/status")
        .then(r => r.json())
        .then(status => {
            const available = status.ready || status.preparable;
            editorBtn.disabled = !available;
            editorBtn.style.opacity = available ? "" : 0.35;
            editorBtn.style.cursor = available ? "" : "not-allowed";
            if (available) {
                editorBtn.title = status.preparable
                    ? "Open editor (prepares layers first, takes a few minutes)"
                    : "Open editor";
            } else {
                editorBtn.title = status.reason || "Editor unavailable in this session";
            }
        })
        .catch(() => {});
}
refreshEditorStatus();

document.getElementById("stretch-slider").addEventListener("input", (e) => {
    setStretch(parseFloat(e.target.value));
});
document.getElementById("polefill-slider").addEventListener("input", (e) => {
    setPoleFill(parseFloat(e.target.value));
});

function resetView() {
    targetLon = 0;
    targetLat = 0;
    targetFov = 75;
    momentum = { x: 0, y: 0 };
    setStretch(1.0);
    setPoleFill(0.0);
}

function toggleFullscreen() {
    if (!document.fullscreenElement) {
        document.documentElement.requestFullscreen();
    } else {
        document.exitFullscreen();
    }
}

function updateAutoRotateUI() {
    document.getElementById("autorotate-indicator").classList.toggle("visible", autoRotate);
}

// Hands over the stitched equirectangular file itself, at whatever resolution it
// was rendered. Distinct from both export paths: this is the file on disk, so it
// carries no viewer edits, and it is the only way to get the panorama out at
// full size without the GPU's texture limit in the way.
function downloadPanorama() {
    const statusEl = document.getElementById("upload-status");
    const msgEl = document.getElementById("upload-message");
    const flash = (text, ms) => {
        msgEl.textContent = text;
        statusEl.classList.add("visible");
        setTimeout(() => statusEl.classList.remove("visible"), ms);
    };

    fetch("/panorama-info")
        .then((r) => r.json())
        .then((info) => {
            if (!info.path) {
                flash("No panorama loaded", 2000);
                return;
            }
            const name = info.path.split(/[\\/]/).pop() || "panorama.jpg";
            const a = document.createElement("a");
            a.href = "/panorama-image";
            a.download = name;
            document.body.appendChild(a);
            a.click();
            a.remove();
            flash(`Downloading ${name}`, 1500);
        })
        .catch((err) => {
            console.error("Download failed:", err);
            flash("Download failed", 2000);
        });
}

// Builds the GPano XMP packet that marks a JPEG as a full equirectangular
// sphere. Keep this in step with _inject_xmp_pano in stitcher.py, which writes
// the same properties onto the stitcher's own output.
function gpanoPacket(width, height, headingDeg) {
    // begin must carry the byte-order mark itself, encoded in the packet's
    // encoding — "\uFEFF" through TextEncoder is EF BB BF.
    return (
        '<?xpacket begin="\uFEFF" id="W5M0MpCehiHzreSzNTczkc9d"?>' +
        '<x:xmpmeta xmlns:x="adobe:ns:meta/">' +
        '<rdf:RDF xmlns:rdf="http://www.w3.org/1999/02/22-rdf-syntax-ns#">' +
        '<rdf:Description rdf:about="" ' +
        'xmlns:GPano="http://ns.google.com/photos/1.0/panorama/" ' +
        'GPano:ProjectionType="equirectangular" ' +
        'GPano:UsePanoramaViewer="True" ' +
        `GPano:CroppedAreaImageWidthPixels="${width}" ` +
        `GPano:CroppedAreaImageHeightPixels="${height}" ` +
        `GPano:FullPanoWidthPixels="${width}" ` +
        `GPano:FullPanoHeightPixels="${height}" ` +
        'GPano:CroppedAreaLeftPixels="0" ' +
        'GPano:CroppedAreaTopPixels="0" ' +
        `GPano:InitialViewHeadingDegrees="${headingDeg}" ` +
        '/></rdf:RDF></x:xmpmeta>' +
        '<?xpacket end="w"?>'
    );
}

// Splices an XMP APP1 segment in directly after the SOI marker. Canvas JPEGs
// carry no metadata of their own, so there is nothing to strip first.
function injectGPano(jpeg, width, height, headingDeg) {
    if (jpeg[0] !== 0xff || jpeg[1] !== 0xd8) return jpeg;

    const encoder = new TextEncoder();
    const signature = encoder.encode("http://ns.adobe.com/xap/1.0/\0");
    const packet = encoder.encode(gpanoPacket(width, height, headingDeg));
    const length = signature.length + packet.length + 2; // includes the length field

    const segment = new Uint8Array(4 + signature.length + packet.length);
    segment[0] = 0xff;
    segment[1] = 0xe1;
    segment[2] = length >> 8;
    segment[3] = length & 0xff;
    segment.set(signature, 4);
    segment.set(packet, 4 + signature.length);

    const out = new Uint8Array(jpeg.length + segment.length);
    out.set(jpeg.subarray(0, 2), 0);
    out.set(segment, 2);
    out.set(jpeg.subarray(2), 2 + segment.length);
    return out;
}

// A true 360 export: the whole sphere at 2:1 with the colour grade and pole
// fill baked in, tagged with GPano so Google Photos, Facebook and VR players
// recognise it. Distinct from exportFullRes, which is a screenshot of the
// current view in whatever projection is on screen and is not a 360 image at
// all. The projection modes and the camera are deliberately ignored here; the
// direction you are facing is recorded as the initial view heading instead of
// being rotated into the pixels, which would tilt the horizon.
function exportEquirect() {
    if (!panoTexture) return;

    const gl = renderer.getContext();
    const maxSize = gl.getParameter(gl.MAX_RENDERBUFFER_SIZE);
    // Even dimensions keep the 2:1 ratio exact, which is what GPano readers
    // check when deciding whether the image covers the full sphere.
    let outW = Math.min(panoTexture.image.width, maxSize);
    outW -= outW % 2;
    const outH = outW / 2;

    const statusEl = document.getElementById("upload-status");
    const msgEl = document.getElementById("upload-message");
    msgEl.textContent = `Exporting 360 ${outW}×${outH}...`;
    statusEl.classList.add("visible");

    requestAnimationFrame(() => {
        try {
            const rt = new THREE.WebGLRenderTarget(outW, outH);
            rt.texture.colorSpace = THREE.SRGBColorSpace;
            renderer.setRenderTarget(rt);
            eqMaterial.uniforms.resolution.value.set(outW, outH);
            renderer.render(eqScene, tpCamera);

            const pixels = new Uint8Array(outW * outH * 4);
            renderer.readRenderTargetPixels(rt, 0, 0, outW, outH, pixels);
            renderer.setRenderTarget(null);
            rt.dispose();

            const canvas = document.createElement("canvas");
            canvas.width = outW;
            canvas.height = outH;
            const ctx = canvas.getContext("2d");
            const img = ctx.createImageData(outW, outH);
            for (let y = 0; y < outH; y++) {
                const src = (outH - 1 - y) * outW * 4;
                img.data.set(pixels.subarray(src, src + outW * 4), y * outW * 4);
            }
            ctx.putImageData(img, 0, 0);

            const heading = Math.round(((lon % 360) + 360) % 360);
            const fail = (err) => {
                console.error("Equirectangular export failed:", err);
                msgEl.textContent = "Export failed";
                setTimeout(() => statusEl.classList.remove("visible"), 2000);
            };
            // Everything past this point is asynchronous and so outside the
            // catch below; without its own handling a failure here just leaves
            // the spinner up forever. toBlob hands back null rather than
            // throwing when it cannot encode, which at this size is a real
            // possibility.
            canvas.toBlob(
                (blob) => {
                    if (!blob) {
                        fail(new Error(`toBlob returned null at ${outW}x${outH}`));
                        return;
                    }
                    blob.arrayBuffer().then((buf) => {
                        const tagged = injectGPano(new Uint8Array(buf), outW, outH, heading);
                        const a = document.createElement("a");
                        a.href = URL.createObjectURL(
                            new Blob([tagged], { type: "image/jpeg" })
                        );
                        a.download = `panorama_equirect_${outW}x${outH}.jpg`;
                        a.click();
                        URL.revokeObjectURL(a.href);
                        msgEl.textContent = "Exported 360!";
                        setTimeout(() => statusEl.classList.remove("visible"), 1200);
                    }).catch(fail);
                },
                "image/jpeg",
                0.95
            );
        } catch (err) {
            console.error("Equirectangular export failed:", err);
            renderer.setRenderTarget(null);
            msgEl.textContent = "Export failed";
            setTimeout(() => statusEl.classList.remove("visible"), 2000);
        }
    });
}

function exportFullRes() {
    if (!panoTexture) return;

    const srcW = panoTexture.image.width;
    const srcH = panoTexture.image.height;
    const gl = renderer.getContext();
    const maxSize = gl.getParameter(gl.MAX_RENDERBUFFER_SIZE);

    // Export at the viewport's aspect ratio in every mode. The tiny-planet and
    // tunnel shaders scale their horizontal extent by resolution.x/resolution.y,
    // so rendering them square — as this used to — re-frames the projection
    // tighter than what is on screen and crops off whatever sat outside the
    // middle square of a wide window.
    const aspect = innerWidth / innerHeight;
    let exportH = Math.min(Math.max(srcH, innerHeight), maxSize);
    let exportW = Math.round(exportH * aspect);
    if (exportW > maxSize) {
        exportW = maxSize;
        exportH = Math.round(exportW / aspect);
    }

    const statusEl = document.getElementById("upload-status");
    const msgEl = document.getElementById("upload-message");
    msgEl.textContent = `Exporting ${exportW}×${exportH}...`;
    statusEl.classList.add("visible");

    requestAnimationFrame(() => {
        try {
            const rt = new THREE.WebGLRenderTarget(exportW, exportH);
            // A render target defaults to no colour space, which turns the
            // linear→sRGB step at the end of both shaders into a no-op and
            // saves a dark JPEG. Rendering to the canvas gets this from
            // renderer.outputColorSpace; a target has to be told.
            rt.texture.colorSpace = THREE.SRGBColorSpace;
            renderer.setRenderTarget(rt);

            if (stereoMode > 0) {
                const savedRes = tpMaterial.uniforms.resolution.value.clone();
                tpMaterial.uniforms.resolution.value.set(exportW, exportH);
                renderer.render(tpScene, tpCamera);
                tpMaterial.uniforms.resolution.value.copy(savedRes);
            } else {
                const exportCam = camera.clone();
                exportCam.aspect = exportW / exportH;
                exportCam.updateProjectionMatrix();
                renderer.render(scene, exportCam);
            }

            const pixels = new Uint8Array(exportW * exportH * 4);
            renderer.readRenderTargetPixels(rt, 0, 0, exportW, exportH, pixels);
            renderer.setRenderTarget(null);
            rt.dispose();

            const canvas = document.createElement("canvas");
            canvas.width = exportW;
            canvas.height = exportH;
            const ctx = canvas.getContext("2d");
            const img = ctx.createImageData(exportW, exportH);
            for (let y = 0; y < exportH; y++) {
                const src = (exportH - 1 - y) * exportW * 4;
                img.data.set(pixels.subarray(src, src + exportW * 4), y * exportW * 4);
            }
            ctx.putImageData(img, 0, 0);

            canvas.toBlob(
                (blob) => {
                    const a = document.createElement("a");
                    a.href = URL.createObjectURL(blob);
                    a.download = `panorama_${STEREO_LABELS[stereoMode]}_${exportW}x${exportH}.jpg`;
                    a.click();
                    URL.revokeObjectURL(a.href);
                    msgEl.textContent = "Exported!";
                    setTimeout(() => statusEl.classList.remove("visible"), 1200);
                },
                "image/jpeg",
                0.95
            );
        } catch (err) {
            console.error("Export failed:", err);
            msgEl.textContent = "Export failed";
            setTimeout(() => statusEl.classList.remove("visible"), 2000);
        }
    });
}

// --- Resize ---
window.addEventListener("resize", () => {
    camera.aspect = innerWidth / innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(innerWidth, innerHeight);
    tpMaterial.uniforms.resolution.value.set(innerWidth * devicePixelRatio, innerHeight * devicePixelRatio);
});

// --- Fade hint ---
setTimeout(() => {
    document.getElementById("info").style.opacity = "0";
}, 4000);

// --- Render loop ---
function animate() {
    requestAnimationFrame(animate);

    if (autoRotate && !isDragging) {
        targetLon += autoRotateSpeed;
    }

    if (!isDragging) {
        targetLon += momentum.x;
        targetLat += momentum.y;
        targetLat = THREE.MathUtils.clamp(targetLat, -LAT_MAX, LAT_MAX);
        momentum.x *= 0.95;
        momentum.y *= 0.95;
        if (Math.abs(momentum.x) < 0.001) momentum.x = 0;
        if (Math.abs(momentum.y) < 0.001) momentum.y = 0;
    }

    lon += (targetLon - lon) * 0.1;
    lat += (targetLat - lat) * 0.1;
    fov += (targetFov - fov) * 0.1;

    if (stereoMode > 0) {
        tpMaterial.uniforms.rotation.value = THREE.MathUtils.degToRad(lon);
        tpMaterial.uniforms.tilt.value = THREE.MathUtils.degToRad(lat);
        tpMaterial.uniforms.zoom.value = 75 / fov;
        renderer.render(tpScene, tpCamera);
    } else {
        camera.fov = fov;
        camera.updateProjectionMatrix();

        const phi = THREE.MathUtils.degToRad(90 - lat);
        const theta = THREE.MathUtils.degToRad(lon);
        camera.lookAt(
            50 * Math.sin(phi) * Math.cos(theta),
            50 * Math.cos(phi),
            50 * Math.sin(phi) * Math.sin(theta)
        );

        renderer.render(scene, camera);
    }
}

animate();

// --- Import / Upload ---
const fileInput = document.getElementById("file-input");
const uploadStatus = document.getElementById("upload-status");
const uploadMessage = document.getElementById("upload-message");
const dropOverlay = document.getElementById("drop-overlay");

function dismissWelcome() {
    const w = document.getElementById("welcome");
    if (w && !w.classList.contains("hidden")) {
        w.classList.add("hidden");
        setTimeout(() => w.remove(), 600);
    }
}

document.getElementById("btn-import").addEventListener("click", () => {
    fileInput.click();
});

document.getElementById("welcome-import")?.addEventListener("click", () => {
    fileInput.click();
});

fileInput.addEventListener("change", () => {
    if (fileInput.files.length > 0) uploadFiles(fileInput.files);
    fileInput.value = "";
});

let dragCounter = 0;
window.addEventListener("dragenter", (e) => {
    e.preventDefault();
    dragCounter++;
    dropOverlay.classList.add("visible");
});
window.addEventListener("dragleave", (e) => {
    e.preventDefault();
    dragCounter--;
    if (dragCounter <= 0) {
        dragCounter = 0;
        dropOverlay.classList.remove("visible");
    }
});
window.addEventListener("dragover", (e) => e.preventDefault());
window.addEventListener("drop", (e) => {
    e.preventDefault();
    dragCounter = 0;
    dropOverlay.classList.remove("visible");
    if (e.dataTransfer.files.length > 0) uploadFiles(e.dataTransfer.files);
});

async function uploadFiles(fileList) {
    const files = Array.from(fileList).filter(f => f.type.startsWith("image/"));
    if (files.length === 0) return;

    const isSingle = files.length === 1;
    uploadMessage.textContent = isSingle
        ? "Loading panorama..."
        : `Stitching ${files.length} images...`;
    uploadStatus.classList.add("visible");

    const form = new FormData();
    for (const f of files) form.append("images", f);

    let success = false;
    try {
        const res = await fetch("/upload", { method: "POST", body: form });
        const data = await res.json();

        if (!res.ok) {
            uploadMessage.textContent = data.error || "Upload failed";
            setTimeout(() => uploadStatus.classList.remove("visible"), 3000);
            // Stitching can fail on images the editor could still work from.
            refreshEditorStatus();
            return;
        }
        success = true;
        uploadMessage.textContent = isSingle ? "Loaded!" : `Stitched ${data.count} images!`;
    } catch (err) {
        console.error("Upload response error, checking if file was saved...", err);
        await new Promise(r => setTimeout(r, 500));
        try {
            const info = await fetch("/panorama-info").then(r => r.json());
            if (info.path) {
                success = true;
                uploadMessage.textContent = "Loaded!";
            }
        } catch {}
    }

    if (success) {
        try {
            const cacheBust = "/panorama-image?t=" + Date.now();
            await loadPanoramaTexture(cacheBust);
            dismissWelcome();
            resetView();
        } catch (e) {
            console.error("Failed to load texture after upload", e);
        }
        refreshEditorStatus();
        setTimeout(() => uploadStatus.classList.remove("visible"), 1200);
    } else {
        uploadMessage.textContent = "Upload failed";
        setTimeout(() => uploadStatus.classList.remove("visible"), 3000);
    }
}
