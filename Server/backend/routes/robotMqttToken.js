// JWT token provider for robot
const express = require("express");
const jwt = require("jsonwebtoken");
const admin = require("firebase-admin");

const router = express.Router();
const db = admin.firestore();

router.post("/robot-mqtt-token", async (req, res) => {
  try {
    const { robotId, mqttPassword } = req.body;

    if (!robotId || !mqttPassword) {
      return res.status(400).json({ error: "missing_fields" });
    }

    // FIXED: correct collection name
    const robotRef = db.collection("robot_private").doc(robotId);
    const robotDoc = await robotRef.get();

    if (!robotDoc.exists) {
      return res.status(404).json({ error: "robot_not_found" });
    }

    const data = robotDoc.data();

    if (data.mqttPassword !== mqttPassword) {
      return res.status(403).json({ error: "invalid_robot_secret" });
    }

    // Create JWT for EMQX
    const token = jwt.sign(
      {
        username: data.mqttUsername,
        robotId: robotId
      },
      process.env.EMQX_JWT_SECRET,
      { expiresIn: "10m" }
    );

    return res.json({
      status: "ok",
      mqttUsername: data.mqttUsername,
      token,
      mqttBroker: "mqtt://server_ip_address:1883"
    });

  } catch (err) {
    console.error("ROBOT MQTT TOKEN ERROR:", err);
    return res.status(500).json({ error: "server_error" });
  }
});

module.exports = router;

