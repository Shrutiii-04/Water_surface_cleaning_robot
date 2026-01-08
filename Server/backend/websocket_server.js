require("dotenv").config();
const admin = require("firebase-admin");
const WebSocket = require("ws");

// Firebase Admin Init
admin.initializeApp({
  credential: admin.credential.cert(require("./serviceAccountKey.json")),
});

const db = admin.firestore();
const PORT = process.env.WEBSOCKET_PORT || 8080;
const wss = new WebSocket.Server({ port: PORT });
console.log(`WebSocket server running on ws://0.0.0.0:${PORT}`);
const robotConnections = new Map();
const userConnections = new Map();

wss.on("connection", async (ws) => {
  console.log("\nNew WebSocket connection");

  ws.isAlive = true;
  ws.authenticated = false;
  ws.role = null;
  ws.robotId = null;
  ws.userUid = null;
  ws.ownerUid = null;

  ws.on("pong", () => (ws.isAlive = true));

  ws.on("message", async (msg, isBinary) => {
    if (isBinary) {
      return handleBinaryFrame(ws, msg);
    }
    try {
      const data = JSON.parse(msg);
      const { type } = data;

      if (type === "auth_robot") {
        await handleRobotAuth(ws, data);
        return;
      }

      if (type === "auth_app") {
        await handleAppAuth(ws, data);
        return;
      }

      if (!ws.authenticated) {
        ws.send(JSON.stringify({ error: "unauthorized" }));
        return;
      }

      await handleAuthedMessage(ws, data);
    } catch (err) {
      console.error("WS parse error:", err);
    }
  });

  ws.on("close", () => {
    console.log("WebSocket closed");
    for (const [robotId, sock] of robotConnections.entries()) {
      if (sock === ws) {
        robotConnections.delete(robotId);
        console.log(`Robot ws removed → ${robotId}`);
      }
    }
    for (const [userUid, sock] of userConnections.entries()) {
      if (sock === ws) {
        userConnections.delete(userUid);
        console.log(`User ws removed → ${userUid}`);
      }
    }
  });
});

// AUTH HANDLERS 
async function handleRobotAuth(ws, data) {
  const { robotId, secretKey } = data;

  if (!robotId || !secretKey) {
    ws.send(JSON.stringify({ error: "missing_fields" }));
    return;
  }

  const robotRef = db.collection("robot_private").doc(robotId);
  const robotDoc = await robotRef.get();

  if (!robotDoc.exists) {
    ws.send(JSON.stringify({ error: "robot_not_found" }));
    return;
  }

  const robotData = robotDoc.data();

  if (robotData.robotSecret !== secretKey) {
    ws.send(JSON.stringify({ error: "invalid_robot_secret" }));
    return;
  }

  // If another socket already registered for this robot, close it
  const existing = robotConnections.get(robotId);
  if (existing && existing !== ws) {
    try {
      existing.close();
    } catch (e) {
      console.error("Error closing existing robot socket:", e);
    }
  }

  ws.authenticated = true;
  ws.role = "robot";
  ws.robotId = robotId;
  ws.ownerUid = robotData.ownerUid; 

  robotConnections.set(robotId, ws);

  console.log(`Robot connected successfully → ${robotId} (owner=${robotData.ownerUid})`);
  ws.send(JSON.stringify({ status: "robot_authenticated" }));
}

async function handleAppAuth(ws, data) {
  const { firebaseToken, robotId } = data;

  if (!firebaseToken || !robotId) {
    ws.send(JSON.stringify({ error: "missing_fields" }));
    return;
  }

  try {
    const decoded = await admin.auth().verifyIdToken(firebaseToken);
    const userUid = decoded.uid;

    const robotDoc = await db.collection("robot_private").doc(robotId).get();
    if (!robotDoc.exists) {
      ws.send(JSON.stringify({ error: "robot_not_found" }));
      return;
    }

    const robotData = robotDoc.data();

    if (robotData.ownerUid !== userUid) {
      ws.send(JSON.stringify({ error: "unauthorized_robot_access" }));
      return;
    }
    const existing = userConnections.get(userUid);
    if (existing && existing !== ws) {
      try {
        existing.close();
      } catch (e) {
        console.error("Error closing existing user socket:", e);
      }
    }

    ws.authenticated = true;
    ws.role = "app";
    ws.userUid = userUid;
    ws.robotId = robotId;

    userConnections.set(userUid, ws);

    console.log(`App connected → user=${userUid}, robot=${robotId}`);
    ws.send(JSON.stringify({ status: "app_authenticated" }));
  } catch (err) {
    console.error("Firebase token error:", err);
    ws.send(JSON.stringify({ error: "invalid_firebase_token" }));
  }
}

// CONTROL MESSAGE HANDLING 
async function handleAuthedMessage(ws, data) {
  if (ws.role === "app") {
    const robotSocket = robotConnections.get(ws.robotId);

    if (!robotSocket) {
      ws.send(JSON.stringify({ error: "robot_not_connected" }));
      return;
    }

    robotSocket.send(JSON.stringify(data));
    return;
  }

  // ROBOT → APP (telemetry, status JSON)
  if (ws.role === "robot") {
    const userUid = ws.ownerUid;
    if (!userUid) {
      return;
    }

    const appSocket = userConnections.get(userUid);
    if (appSocket) {
      appSocket.send(JSON.stringify(data));
    }
    return;
  }
}

// VIDEO FRAME HANDLER 
function handleBinaryFrame(ws, frameBuffer) {
  if (!ws.authenticated || ws.role !== "robot") return;

  const userUid = ws.ownerUid;
  if (!userUid) return;

  const appSocket = userConnections.get(userUid);

  if (appSocket) {
    appSocket.send(frameBuffer, { binary: true });
  }
}

// KEEPALIVE 
setInterval(() => {
  wss.clients.forEach((ws) => {
    if (!ws.isAlive) {
      return ws.terminate();
    }
    ws.isAlive = false;
    ws.ping();
  });
}, 30000);
