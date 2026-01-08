require("dotenv").config();
const express = require("express");
const admin = require("firebase-admin");
const cors = require("cors");
const helmet = require("helmet");

const app = express();
app.use(express.json());
app.use(cors());
app.use(helmet());

// Init Firebase Admin
admin.initializeApp({
  credential: admin.credential.cert(require("./serviceAccountKey.json")),
});

const db = admin.firestore();
app.use("/", require("./routes/mqttToken"));
app.use("/", require("./routes/robotMqttToken"));
app.get("/", (req, res) => {
  res.send("Backend is running");
});

// Pairing api
app.post("/pair", async (req, res) => {
  try {
    const { userUid, robotId, pairCode } = req.body;

    if (!userUid || !robotId || !pairCode) {
      return res.status(400).json({ error: "missing_fields" });
    }

    const robotRef = db.collection("robot_private").doc(robotId);
    const robotDoc = await robotRef.get();

    if (!robotDoc.exists) {
      return res.status(404).json({ error: "robot_not_found" });
    }

    const data = robotDoc.data();

    if (data.ownerUid) {
      return res.status(403).json({ error: "robot_already_paired" });
    }

    if (data.pairCode !== pairCode) {
      return res.status(403).json({ error: "invalid_pair_code" });
    }

    await robotRef.update({
      ownerUid: userUid,
      pairCode: null,
      pairedAt: admin.firestore.FieldValue.serverTimestamp(),
    });

    await db.collection("users").doc(userUid).set(
      {
        robotId,
        pairedAt: admin.firestore.FieldValue.serverTimestamp(),
      },
      { merge: true }
    );

    return res.json({ status: "paired", robotId });
  } catch (err) {
    console.error("PAIR ERROR:", err);
    return res.status(500).json({ error: "server_error" });
  }
});
const PORT = process.env.PORT || 3000;
app.listen(PORT, () => console.log(`Backend listening on port ${PORT}`));
