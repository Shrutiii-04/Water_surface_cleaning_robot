from flask import Flask, request, jsonify

app = Flask(__name__)
last_mission = {}

@app.route("/send_mission", methods=["POST"])
def send_mission():
    global last_mission
    data = request.json
    if not data or "polygon" not in data:
        return jsonify({"status": "error", "message": "Invalid mission"}), 400

    last_mission = data
    print("\nNew Mission Received:", last_mission)
    return jsonify({"status": "success", "data": last_mission})

@app.route("/get_mission", methods=["GET"])
def get_mission():
    return jsonify({"status": "success", "data": last_mission})

if __name__ == "__main__":
    print("Mission Server Running...")
    app.run(host="0.0.0.0", port=5000)
