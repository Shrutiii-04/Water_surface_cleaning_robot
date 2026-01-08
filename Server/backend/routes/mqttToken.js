// JWT token provider user
const express = require("express");
const jwt = require("jsonwebtoken");
const admin = require("firebase-admin");

const router = express.Router();
const db = admin.firestore();

router.post("/mqtt-token", async (req, res) => {
  const { userUid, robotId } = req.body;

  if (!userUid || !robotId) {
    return res.status(400).json({ error: "missing_fields" });
  }

  // Check robot document
  const robotRef = db.collection("robot_private").doc(robotId);
  const robotDoc = await robotRef.get();

  if (!robotDoc.exists) {
    return res.status(404).json({ error: "robot_not_found" });
  }

  const data = robotDoc.data();

  // Verify ownership
  if (data.ownerUid !== userUid) {
    return res.status(403).json({ error: "unauthorized" });
  }

  // Generate JWT for MQTT
  const token = jwt.sign(
    {
      username: data.mqttUsername,
      robotId: robotId,
    },
    process.env.EMQX_JWT_SECRET,     
    { expiresIn: "10m" }          
  );

  return res.json({
    token,
    mqttUsername: data.mqttUsername,
    mqttBroker: "mqtt://server_ip_address",
  });
});

module.exports = router;
