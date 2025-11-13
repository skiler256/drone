

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
cameraViewport.position.set(0, 14, 0); // vue isométrique

cameraViewport.lookAt(0, 0, 0);
cameraViewport.rotation.z = Math.PI / -2;
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
        droneModel.traverse((child) => {
        if (child.isMesh) {
            // Si le mesh a un matériau, on change sa couleurs
            if (child.material) {
                child.material.color.set(0xFFFFFF); // Rouge par exemple
            }
        }
    });
}, undefined, function (error) {
    console.error('Erreur lors du chargement du modèle :', error);
});
const textureLoader = new THREE.TextureLoader();
textureLoader.load('texture/fondCiel.jpg', function (texture) {
  viewport.background = texture;
});


const light = new THREE.DirectionalLight(0xffffff, 1);
const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
viewport.add(ambientLight);
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

const socket = new WebSocket("ws://192.168.1.24:9001/");

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

    document.getElementById("console").innerHTML = sysData.state3D.pos[2];
    renderGPS(sysData.sensor.gps);
    updateModule(sysData.module)

const calzBt = document.getElementById('calZbt');

    switch(sysData.state3D.INSstate){
      case 0:
      calzBt.style.backgroundColor = "red";
      break;
      case 1:
      calzBt.style.backgroundColor = "orange";
      break;
      case 2:
      calzBt.style.backgroundColor = "green";
      break;

    }


    updateGauge(sysData);

  console.log("CAP ", sysData.sensor);
  
  console.log("state ", sysData);
});

function switchView(){
  const videoView = document.getElementById('videoView');
  if(videoView.style.visibility == 'hidden') videoView.style.visibility ='visible';
    else videoView.style.visibility = 'hidden';

    const videoLegend= document.getElementById('videoLegend');
    if(videoLegend.style.visibility == 'hidden') videoLegend.style.visibility ='visible';
      else videoLegend.style.visibility = 'hidden';
}

const gimballMode = document.getElementById('gimballMode');

gimballMode.addEventListener('change', function() {
  socket.send("GIM" +this.value );
});

function updateModule(moduleState){
  const btClass = document.querySelectorAll(".MODULEbt");

  for(const bt of btClass){
    if(moduleState[bt.id]) bt.style.backgroundColor = "green";
    else bt.style.backgroundColor = "red";
  }
}

async function setINS(){
  const pageDIV= document.getElementById("pageDIV");
  pageDIV.style.visibility = "visible";

  const response = await fetch("INSparam.html");
  const text = await response.text();

  pageDIV.innerHTML = "";
  pageDIV.innerHTML = text;
}


function sendINSparam() {
  let data = {};
  data.refreshRate = parseInt(document.getElementById("predResfreshRate").value);
  data.zRefreshRate = parseInt(document.getElementById("zRefreshRate").value);
  data.alphaHeading = parseFloat(document.getElementById("AlphaHeading").value);
  data.NGPSattempt = parseInt(document.getElementById("NBgps").value);
  data.NmoyGPScalib = parseInt(document.getElementById("NBzmoy").value);
  data.baseAltitude = parseFloat(document.getElementById("baseAlt").value);

  console.log(data);

  socket.send("IPA" + JSON.stringify(data));
  document.getElementById('pageDIV').style.visibility = 'hidden';
}

function updateGauge(data){
  const needle_TEMP = document.getElementById("needle_TEMP");
  gsap.to(needle_TEMP, { rotation: 90+(330*data.perf.CPUtemp/100), duration: 0.5, ease: "power1.out" });

  const needle_VS = document.getElementById("needle_VS");
  gsap.to(needle_VS, { rotation: 180+(165*data.state3D.vel[2]), duration: 0.5, ease: "power1.out" });

  const needle_ALT = document.getElementById("needle_ALT");
  gsap.to(needle_ALT, { rotation: 105+(330*data.state3D.pos[2]/10), duration: 0.5, ease: "power1.out" });

  {
    const gauge = document.getElementById("gauge_ACC");
    const needle = document.getElementById("needle_ACC");
    const maxMoveX = gauge.clientWidth / 2;  
    const maxMoveY = gauge.clientHeight / 2; 

    accX = Math.max(-9.81, Math.min(9.81, data.state3D.accNED[1]));
    accY = Math.max(-9.81, Math.min(9.81, data.state3D.accNED[0]));

    const x = (accX / 9.81) * maxMoveX;
    const y = -(accY / 9.81) * maxMoveY;

    // Animer avec GSAP
    gsap.to(needle, { duration: 0.1, x: x, y: y });

  }
  const Xcoord = document.getElementById("Xcoord");
  const Ycoord = document.getElementById("Ycoord");

  Xcoord.innerText = "Nord : " +Math.round( data.state3D.pos[0]* 10)/10 + "m";
  Ycoord.innerText = "Est : "+ Math.round( data.state3D.pos[1]*10)/10 + "m";

}