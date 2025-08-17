

async function addSatellite(color) {
    const response = await fetch("texture/satellite.svg");
    let svgText = await response.text();
  
const container = document.getElementById("satIcons");
container.innerHTML = "";

    const wrapper = document.createElement("div");
    wrapper.innerHTML = svgText;
  
    // Récupérer la vraie balise <svg>
    const svg = wrapper.querySelector("svg");
    if (!svg) {
      console.error("SVG non trouvé !");
      return;
    }
  
    // appliquer couleur et taille
    svg.style.color = color;       // fonctionne si fill="currentColor" dans SVG
    svg.style.width = "50px";
    svg.style.height = "50px";
  
    container.appendChild(svg);
  }

let satChart = null;


function renderGPS(gps) {
    const satellites = gps.sats;

    if(gps.gpsFixOk)  addSatellite("green");
    else  addSatellite("red");

    const labels = satellites.map(s => s.ID);
    const strengths = satellites.map(s => s.strenght);
    const colors = satellites.map(s => s.quality >= 4 ? 'green' : 'red');

    if (satChart) {
        satChart.data.labels = labels;
        satChart.data.datasets[0].data = strengths;
        satChart.data.datasets[0].backgroundColor = colors;
        satChart.update();
    } else {
        satChart = new Chart(document.getElementById('satsCanva'), {
            type: 'bar',
            data: {
                labels: labels,
                datasets: [{
                    data: strengths,
                    backgroundColor: colors
                }]
            },
            options: {
                responsive: true,
                plugins: {
                    legend: { display: false },   // pas de légende
                    title: { display: false }     // pas de titre
                },
                scales: {
                    x: {
                        title: { display: false }, // pas de titre
                        ticks: { color: "white" }  // affiche seulement numéros ID
                    },
                    y: {
                        title: { display: false }, // pas de titre
                        ticks: { display: false }, // enlève chiffres Y
                        grid: { display: false }   // enlève la grille
                    }
                }
            }
        });
    }
}
