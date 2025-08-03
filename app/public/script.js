alert("lance");


const socket = new WebSocket("ws://jean-drone.local:9001/");


socket.addEventListener("open", (event) => {
  socket.send("Hello Server!");
});

// Listen for messages
socket.addEventListener("message", (event) => {
    document.getElementById("console").innerHTML = event.data;
  console.log("Message from server ", event.data);
});