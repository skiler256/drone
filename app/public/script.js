alert("lance");

let scene, camera, renderer, sphere;
let textureUrl = "texture/horizon.jpg"; // ← remplace par ton fichier

init();
animate();

function init() {
    scene = new THREE.Scene();
    const width = 500;
    const height = 500;

    camera = new THREE.PerspectiveCamera(30, width / height, 0.1, 1000);
    camera.position.z = 6;

    renderer = new THREE.WebGLRenderer({ canvas: document.getElementById('horizonCanvas') });
    renderer.setSize(width, height,true);

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

    window.addEventListener('resize', onWindowResize, false);
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



const socket = new WebSocket("ws://jean-drone.local:9001/");

let sysData = {};



socket.addEventListener("open", (event) => {
  socket.send("Hello Server!");
});

// Listen for messages
socket.addEventListener("message", (event) => {
    document.getElementById("console").innerHTML = event.data;
    sysData = JSON.parse(event.data);
    updateAttitude(sysData.state3D.att);
  console.log("CAP ", sysData.state3D.att);
});
