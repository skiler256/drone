const express = require("express");

const app = express();
const PORT = 8080;


app.use(express.static("../app/public"));

app.listen(PORT, () => {
    console.log(`Serveur statique en ligne sur http://localhost:${PORT}`);
});
