alert("lance");

let scene, camera, renderer, sphere;
let textureUrl = "texture/horizon.jpg"; // ← remplace par ton fichier

init();
animate();

function init() {
    scene = new THREE.Scene();
    const width = 500;
    const height = 500;

    camera = new THREE.PerspectiveCamera(5, width / height, 0.1, 1000);
    camera.position.z = 25;

    renderer = new THREE.WebGLRenderer({ canvas: document.getElementById('horizonCanvas') });
    renderer.setSize(width, height,false);

    const loader = new THREE.TextureLoader();
    loader.load(textureUrl, function (texture) {
        const geometry = new THREE.SphereGeometry(1, 64, 64);
        const material = new THREE.MeshBasicMaterial({
            map: texture,
            side: THREE.BackSide // ← pour voir l'intérieur
        });
        sphere = new THREE.Mesh(geometry, material);
        scene.add(sphere);
    });

    // window.addEventListener('resize', onWindowResize, false);
}

function onWindowResize() {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
}

function animate() {
    requestAnimationFrame(animate);
    renderer.render(scene, camera);
}
const degToRad = angleDeg => angleDeg * Math.PI / 180;
// Pour recevoir les données de rotation
function updateAttitude(att) {
  if (!sphere) return;

  // On lit : [roll, pitch, yaw]
  const roll  = degToRad(att[0]);  // rotation autour de l’axe X
  const pitch = degToRad(att[1]);  // rotation autour de l’axe Y
  const yaw   = degToRad(att[2]+90);  // rotation autour de l’axe Z

  // Appliquer YAW (Z), puis PITCH (X), puis ROLL (Y) : ordre ZXY
  const euler = new THREE.Euler(-pitch, yaw, roll, 'ZXY');
  sphere.setRotationFromEuler(euler);
}



const viewportCanva = document.getElementById('viewport');

const viewport = new THREE.Scene();
const cameraViewport = new THREE.PerspectiveCamera(45, viewportCanva.width / viewportCanva.height, 0.1, 1000);
cameraViewport.position.set(4, 4, 3); // vue isométrique
cameraViewport.lookAt(0, 0, 0);

const rendererViewport = new THREE.WebGLRenderer({ canvas: viewportCanva });
rendererViewport.setSize(viewportCanva.clientWidth, viewportCanva.clientHeight, false);

const axesHelper = new THREE.AxesHelper(1);
viewport.add(axesHelper);
const gridHelper = new THREE.GridHelper(20, 20);
viewport.add(gridHelper);

let droneModel;

const loader = new THREE.GLTFLoader();
loader.load('texture/drone3D.glb', function (gltf) {
    droneModel = gltf.scene;
    droneModel.scale.set(1, 1, 1); // ajuste si besoin
    viewport.add(droneModel);
}, undefined, function (error) {
    console.error('Erreur lors du chargement du modèle :', error);
});


const light = new THREE.DirectionalLight(0xffffff, 1);
light.position.set(5, 10, 7);
viewport.add(light);

function animateViewport() {
  requestAnimationFrame(animateViewport);
  rendererViewport.render(viewport, cameraViewport);
}

function updateAttitudeViewport(att) {
  if (!droneModel) return;

  // On lit : [roll, pitch, yaw]
  const roll  = degToRad(att[0]);  // rotation autour de l’axe X
  const pitch = degToRad(att[1]);  // rotation autour de l’axe Y
  const yaw   = degToRad(att[2]);  // rotation autour de l’axe Z

  // Appliquer YAW (Z), puis PITCH (X), puis ROLL (Y) : ordre ZXY
  const euler = new THREE.Euler(-roll, -yaw,pitch , 'YZX');
  droneModel.setRotationFromEuler(euler);
}
function updateDronePOS(pos){
  if (!droneModel) return;
  droneModel.position.set(pos[0],pos[2],pos[1]);
}
animateViewport();

const socket = new WebSocket("ws://jean-drone.local:9001/");

let sysData = {};



socket.addEventListener("open", (event) => {
  socket.send("Hello Server!");
});

// Listen for messages
socket.addEventListener("message", (event) => {

    sysData = JSON.parse(event.data);
    updateAttitude(sysData.state3D.att);
    updateAttitudeViewport(sysData.state3D.att)
    updateDronePOS(sysData.state3D.pos);

    document.getElementById("console").innerHTML = sysData.events;
  console.log("CAP ", sysData.state3D.pos);
});
