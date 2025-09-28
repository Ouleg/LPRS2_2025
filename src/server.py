from flask import Flask, jsonify
import json

app = Flask(__name__)

@app.route('/data')
def get_data():
    with open("task_data.json") as f:
        return jsonify(json.load(f))

app.run(host="0.0.0.0", port=5000)
